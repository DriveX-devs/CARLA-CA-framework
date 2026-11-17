#!/usr/bin/env bash
# Packet-loss launcher, kept for the campaigns and scripts that call it:
# run_extended.sh itself now takes the two settings this variant added,
#
#   INJECTED_LOSS_PERC   percentage of the packets that reach their
#                        destination and are discarded there on purpose
#   BRIDGE_RNG_RUN       ns-3 RngRun of the bridge (the loss realisation)
#
# so this is run_extended.sh with those two variables required to be set
# explicitly (default 0 % and run 1).
#
# The injected loss is applied by the bridge at the destination application
# socket: the packet really traverses the simulated air interface (so radio
# load and the latency distribution are unchanged) and is then discarded and
# replied as "lost" instead of being handed back to the application.
export INJECTED_LOSS_PERC=${INJECTED_LOSS_PERC:-0}
export BRIDGE_RNG_RUN=${BRIDGE_RNG_RUN:-1}
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_extended.sh" "$@"
