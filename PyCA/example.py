"""
Minimal usage example for the t2c Collision Avoidance module.

Scenario (one representative case):
    Two cars approach the same intersection, placed at (0, 0), on
    perpendicular roads:
    - "carA" drives EAST at 12 m/s, starting 60 m west of the intersection
    - "carB" drives NORTH starting 50 m south of the intersection at 10 m/s,
      gently braking (-0.4 m/s^2)
    Neither car expose its acceleration in the updates, so constant velocity
    is assumed.

    Their paths cross but they do not meet exactly at (0,0) at the same
    instant -> we simulate a near miss, so that s2c is non-zero (and, since carB
    updates do not report its acceleration, s2c changes from update to update).

We suppose to receive a fresh EntityState update for both cars every 200 ms.
At every update we run CheckCollision() on the latest pair of states and print
the result. In parallel, the same states are fed to a CollisionAvoidanceService
configured with a warning callback, to show how to get notified of each risk.
"""

import time

from t2c import t2c, EntityState, CollisionAvoidanceService

UPDATE_PERIOD_s = 0.2
SIM_DURATION_s = 10.0

# Create the collision detector
# T2C_th is the Time-to-Collision threshold: only closest approaches expected
# within ~4.8 s (scaled with speed inside CheckCollision) are considered risky
detector = t2c(node_id="example", T2C_th=4.8)


# Warning callback: invoked once per CollisionWarning found by the
# CollisionAvoidanceService (the alternative, database-based front end)
def on_warning(w):
    pass
    # print(f"COLLISION WARNING: {w.id} and {w.other_id}: {w.collision_type}, t2c={w.t2c:.2f} s, s2c={w.s2c:.2f} m")


# The service keeps the latest state of every entity and runs the check on
# each update(); every detected risk is passed to the callback above
cas = CollisionAvoidanceService(on_warning=on_warning)

print(f"{'time':>5} | {'carA pos':>16} | {'carB pos':>16} | "
      f"{'t2c [s]':>8} | {'s2c [m]':>8} | output_str")
print("-" * 78)

t = 0.0
while t <= SIM_DURATION_s:
    # ------------------------------------------------------------------
    # 1) A new state update arrives for each car (every 200 ms).
    #    In a real deployment these would come from CAMs or sensors;
    #    here we just compute where each car is at time t.
    #    Positions are in meters (projected Cartesian, x = East, y = North),
    #    heading in degrees clockwise from North (90 = East, 0 = North),
    #    following the ETSI format.
    # ------------------------------------------------------------------
    carA = EntityState.from_direct(
        station_id="carA",
        x=-60.0 + 12.0 * t,
        y=0.0,
        speed_ms=12.0,
        heading_deg=90.0,
    )
    # carB is actually braking at -0.4 m/s^2, but its updates carry no
    # acceleration (acc_lon left as None, like a CAM with that field
    # "unavailable"): the detector assumes constant velocity, so the predicted
    # closest approach (s2c) shifts a little at every fresh update
    carB = EntityState.from_direct(
        station_id="carB",
        x=0.0,
        y=-50.0 + 10.0 * t - 0.5 * 0.4 * t * t,
        speed_ms=10.0 - 0.4 * t,
        heading_deg=0.0,
    )

    # ------------------------------------------------------------------
    # 2) Run the collision check on the latest pair of states.
    #    CheckCollision takes raw kinematics: position (x, y), velocity
    #    components (vx, vy) and acceleration components (ax, ay) of both
    #    entities. EntityState computes vx/vy/ax/ay for us from speed,
    #    heading and (optional) accelerations.
    #
    #    It returns (t2c, s2c):
    #      - (t2c, s2c) both >= 0 -> collision risk! The two entities get
    #        within s2c meters of each other in t2c seconds.
    #      - (-1, -1) -> no risk detected for this update.
    # ------------------------------------------------------------------
    t2c_val, s2c_val = detector.CheckCollision(
        carA.x, carA.y, carA.vx, carA.vy, carA.ax, carA.ay,
        carB.x, carB.y, carB.vx, carB.vy, carB.ax, carB.ay,
    )

    # ------------------------------------------------------------------
    # 3) React to the result
    # ------------------------------------------------------------------
    if t2c_val >= 0:
        output_str = f"COLLISION RISK -> warn both cars"
        t2c_str, s2c_str = f"{t2c_val:8.2f}", f"{s2c_val:8.2f}"
    else:
        output_str = "No collision risk"
        t2c_str, s2c_str = f"{'-':>8}", f"{'-':>8}"

    print(f"{t:5.1f} | ({carA.x:6.1f}, {carA.y:5.1f}) | "
          f"({carB.x:6.1f}, {carB.y:5.1f}) | {t2c_str} | {s2c_str} | {output_str}")

    # ------------------------------------------------------------------
    # 4) Same states through the CollisionAvoidanceService: update() stores
    #    each state and checks it against all the others; any risk found is
    #    delivered to the on_warning callback (printed as "[callback] ...").
    # ------------------------------------------------------------------
    cas.update(carA, now=t)
    cas.update(carB, now=t)

    # Simulate next update in 200 ms
    t += UPDATE_PERIOD_s
    time.sleep(UPDATE_PERIOD_s)


# ------------------------------------------------------------------
# Animate the vehicle movements.
# This function recomputes the two trajectories with the same kinematic
# formulas used in the loop above and plays a top-down animation:
# the two cars move along their paths, leaving a trail behind, while the
# title reports the elapsed time and the current CheckCollision() verdict
# (red while a collision risk is detected).
# ------------------------------------------------------------------
def show_movements(duration=SIM_DURATION_s, step=UPDATE_PERIOD_s,
                   gif_name="movements.gif"):
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation, PillowWriter
    except ImportError:
        print("matplotlib not installed (pip3 install matplotlib): "
              "cannot show the movements animation.")
        return

    # Rebuild the trajectories (same formulas as in the simulation loop)
    times = [i * step for i in range(int(duration / step) + 1)]
    pa = [(-60.0 + 12.0 * t, 0.0) for t in times]                       # carA
    pb = [(0.0, -50.0 + 10.0 * t - 0.5 * 0.4 * t * t) for t in times]   # carB

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(-65, 65)
    ax.set_ylim(-55, 35)
    ax.set_xlabel("x = East [m]")
    ax.set_ylabel("y = North [m]")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.plot(0, 0, "x", color="red", markersize=12, markeredgewidth=3,
            label="intersection (0, 0)")

    # Trails (lines that grow frame by frame) and the two moving cars (dots)
    trail_a, = ax.plot([], [], "-", color="tab:blue",
                       label="carA (east, 12 m/s)")
    trail_b, = ax.plot([], [], "-", color="tab:orange",
                       label="carB (north, braking)")
    dot_a, = ax.plot([], [], "o", color="tab:blue", markersize=10)
    dot_b, = ax.plot([], [], "o", color="tab:orange", markersize=10)
    title = ax.set_title("")
    ax.legend(loc="lower right")

    def update_frame(i):
        t = times[i]
        # Move the cars and extend the trails
        trail_a.set_data([p[0] for p in pa[:i + 1]], [p[1] for p in pa[:i + 1]])
        trail_b.set_data([p[0] for p in pb[:i + 1]], [p[1] for p in pb[:i + 1]])
        dot_a.set_data([pa[i][0]], [pa[i][1]])
        dot_b.set_data([pb[i][0]], [pb[i][1]])

        # Same collision check as in the loop, shown live in the title
        # (constant-velocity states, since the updates expose no acceleration)
        vB = 10.0 - 0.4 * t
        t2c_val, s2c_val = detector.CheckCollision(
            pa[i][0], pa[i][1], 12.0, 0.0, 0.0, 0.0,
            pb[i][0], pb[i][1], 0.0, vB, 0.0, 0.0,
        )
        if t2c_val >= 0:
            title.set_text(f"t = {t:4.1f} s | COLLISION RISK: "
                           f"t2c = {t2c_val:.2f} s, s2c = {s2c_val:.2f} m")
            title.set_color("red")
        else:
            title.set_text(f"t = {t:4.1f} s | no collision risk")
            title.set_color("black")
        return trail_a, trail_b, dot_a, dot_b, title

    # One frame per update -> the animation plays in (more or less) real time
    anim = FuncAnimation(fig, update_frame, frames=len(times),
                         interval=step * 1000, repeat=True)

    # anim.save(gif_name, writer=PillowWriter(fps=int(1 / step)))
    # print(f"Movements animation saved to {gif_name}")
    plt.show()

show_movements()