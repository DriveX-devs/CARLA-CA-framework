"""
test_suite.py - validation for the Python T2C / CAS port.
Automatically generated with Clade Opus to perform external validation.

1. Validates TimeToCollision against a brute-force numerical minimisation
   of D(t) (the squared distance between the two trajectories).
2. Runs the cross-road-only CAS (Algorithm 1) on cross-road, diverging,
   stale-state and rear-end-filtering scenarios.
3. Shows CheckCollision (t2c + s2c in one call, speed-adaptive t2c threshold).
"""

import math
from t2c import (t2c, EntityState, CollisionAvoidanceService, NO_COLLISION)


def numeric_t2c(v, w, t_max=60.0, steps=600000):
    """Brute-force argmin of D(t) over [0, t_max]."""
    best_t, best_d = None, float("inf")
    for i in range(steps + 1):
        t = t_max * i / steps
        dx = (v.x + v.vx * t + 0.5 * v.ax * t * t) - (w.x + w.vx * t + 0.5 * w.ax * t * t)
        dy = (v.y + v.vy * t + 0.5 * v.ay * t * t) - (w.y + w.vy * t + 0.5 * w.ay * t * t)
        d = dx * dx + dy * dy
        if d < best_d:
            best_d, best_t = d, t
    return best_t, math.sqrt(best_d)


def check(name, v, w):
    engine = t2c(T2C_th=4.8)
    analytic = engine.TimeToCollision(v.x, v.y, v.vx, v.vy, v.ax, v.ay,
                                      w.x, w.y, w.vx, w.vy, w.ax, w.ay)
    num_t, num_d = numeric_t2c(v, w)
    s2c = engine.computes2c(v.x, v.y, v.vx, v.vy, v.ax, v.ay,
                            w.x, w.y, w.vx, w.vy, w.ax, w.ay,
                            analytic) if analytic >= 0 else float("nan")
    print(f"{name:38s} t2c={analytic:8.4f}  s2c={s2c:8.4f}  "
          f"| numeric argmin t={num_t:8.4f}, min dist={num_d:8.4f}")
    return analytic, s2c


print("=== TimeToCollision validation (analytic vs numeric) ===")

# 1 Perpendicular crossing, constant speed, exact geometric collision
v = EntityState("veh1", x=-50, y=0, speed_ms=10, heading_deg=90)   # eastbound
w = EntityState("veh2", x=0, y=-50, speed_ms=10, heading_deg=0)    # northbound
check("cross-road, const speed, head-on", v, w)

# 2 Perpendicular crossing with accelerations (cubic solver, Q != 0 path)
v = EntityState("veh1", x=-60, y=0, speed_ms=8, heading_deg=90, acc_lon=1.0)
w = EntityState("veh2", x=0, y=-45, speed_ms=12, heading_deg=0, acc_lon=-0.5)
check("cross-road, accelerating", v, w)

# 3 Rear-end: same lane, follower faster and braking less (Q ~ 0 path)
v = EntityState("veh1", x=0, y=0, speed_ms=15, heading_deg=0, acc_lon=0.0)
w = EntityState("veh2", x=0, y=40, speed_ms=8, heading_deg=0, acc_lon=-1.0)
check("rear-end, follower approaching", v, w)

# 4 Diverging vehicles -> expect NO_COLLISION (-1)
v = EntityState("veh1", x=0, y=0, speed_ms=10, heading_deg=90)
w = EntityState("veh2", x=-20, y=0, speed_ms=10, heading_deg=270)
t, _ = check("diverging (expect -1)", v, w)
assert t == NO_COLLISION

print("\n=== CAS (Algorithm 1) ===")
cas = CollisionAvoidanceService()   # alpha_th=17 deg, t2c_th=4.8 s, s2c_th=4.2 m

# track vehicle w heading north towards the intersection at (0,0)
w_state = EntityState("w", x=0, y=-40, speed_ms=13, heading_deg=0)
cas.update(w_state, now=100.0)

# state of v heading east, on collision course
v_state = EntityState("v", x=-40, y=0, speed_ms=13, heading_deg=90)
warnings = cas.update(v_state, now=100.2)
for warn in warnings:
    print(f"DENM -> ({warn.id}, {warn.other_id}) | {warn.collision_type} | "
          f"t2c={warn.t2c:.2f} s, s2c={warn.s2c:.2f} m")
assert len(warnings) == 1

# same geometry but far away in time -> t2c > threshold, no warning
w_far = EntityState("w2", x=0, y=-200, speed_ms=13, heading_deg=0)
cas.update(w_far, now=100.3)
v_far = EntityState("v2", x=-200, y=0, speed_ms=13, heading_deg=90)
assert cas.update(v_far, now=100.4) == []
print("far-away pair correctly ignored (t2c > t2c_th / out of range)")

# stale entry (> 3 s old) must be ignored
w_old = EntityState("w_old", x=0, y=-40, speed_ms=13, heading_deg=0)
cas.update(w_old, now=50.0)
v_new = EntityState("v3", x=-40, y=0, speed_ms=13, heading_deg=90)
res = cas.update(v_new, now=100.5)
assert all(x.other_id != "w_old" for x in res)
print("stale (>3 s) map entry correctly ignored")

# Edge configuration: observations are projected from their transmit time to
# the common CA evaluation time and expire after the fixed 500 ms TTL.
edge_cas = CollisionAvoidanceService(stale_after_s=0.5)
moving = EntityState("moving", x=0, y=0, speed_ms=10, heading_deg=90)
projected = edge_cas._state_at(
    EntityState("moving", x=0, y=0, speed_ms=10, heading_deg=90,
                timestamp=200.0),
    200.4,
)
assert math.isclose(projected.x, 4.0, abs_tol=1e-9)
assert math.isclose(projected.y, 0.0, abs_tol=1e-9)
edge_cas.update(moving, now=200.4, observation_time=200.0)
assert edge_cas.get("moving") is not None
newer = EntityState("ordered", x=5, y=0, speed_ms=0, heading_deg=0)
edge_cas.update(newer, now=200.4, observation_time=200.3)
older = EntityState("ordered", x=1, y=0, speed_ms=0, heading_deg=0)
edge_cas.update(older, now=200.4, observation_time=200.1)
assert edge_cas.get("ordered").x == 5
too_old = EntityState("too_old", x=0, y=0, speed_ms=0, heading_deg=0)
edge_cas.update(too_old, now=201.0, observation_time=200.49)
assert edge_cas.get("too_old") is None
print("edge states projected to evaluation time and fixed 500 ms TTL enforced")

# Rear-end pairs are classified for diagnostics but always skipped.
lead = EntityState("lead", x=0, y=30, speed_ms=5, heading_deg=0)
cas.update(lead, now=100.6)
follower = EntityState("foll", x=0, y=0, speed_ms=14, heading_deg=5)
res = cas.update(follower, now=100.7)
assert all(x.other_id != "lead" for x in res)
risk, t_rear, s_rear, rear_type = cas.check_pair(follower, lead)
assert not risk and t_rear == NO_COLLISION and math.isnan(s_rear)
assert rear_type == CollisionAvoidanceService.REAR_END
print("rear-end pair correctly classified and excluded from CA")

print("\n=== CheckCollision (t2c + s2c in one call) ===")
detector = t2c(T2C_th=4.8)   # base t2c threshold [s]; scaled up with speed


def check_collision(name, v, w):
    t2c_val, s2c_val = detector.CheckCollision(
        v.x, v.y, v.vx, v.vy, v.ax, v.ay,
        w.x, w.y, w.vx, w.vy, w.ax, w.ay)
    if t2c_val == NO_COLLISION:
        print(f"{name:38s} no collision")
    else:
        print(f"{name:38s} COLLISION  t2c={t2c_val:6.2f} s, s2c={s2c_val:6.2f} m")
    return t2c_val, s2c_val


# on collision course -> risk reported
v = EntityState("v", x=-40, y=0, speed_ms=13, heading_deg=90)
w = EntityState("w", x=0, y=-40, speed_ms=13, heading_deg=0)
tc, sc = check_collision("cross-road, on collision course", v, w)
assert tc != NO_COLLISION

# far away in time -> no collision
v = EntityState("v", x=-200, y=0, speed_ms=13, heading_deg=90)
w = EntityState("w", x=0, y=-200, speed_ms=13, heading_deg=0)
tc, sc = check_collision("cross-road, far away", v, w)
assert tc == NO_COLLISION

# crossing but wide miss (safe closest approach) -> no collision
v = EntityState("v", x=-40, y=0, speed_ms=13, heading_deg=90)
w = EntityState("w", x=0, y=-80, speed_ms=13, heading_deg=0)
tc, sc = check_collision("cross-road, wide miss", v, w)
assert tc == NO_COLLISION

print("\nAll checks passed.")
