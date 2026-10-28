#!/usr/bin/env bash
# Drive Qualcomm's unmodified simple_motion node through its keyboard commands and measure the base while each command
# is held. Run inside a container with the qrb_amr_sim simulator up. Measurement starts 0.5 s after each key (the
# controller's acceleration limits ramp the base up first), so the reported rate is the steady-state one.
WINDOW=${WINDOW:-3}
here=$(dirname "$0")
fifo=$(mktemp -u); mkfifo "$fifo"
python3 "$here/qualcomm_simple_motion.py" < "$fifo" > /tmp/simple_motion.log 2>&1 &
exec 3> "$fifo"
sleep 3
echo "--- i (forward 0.5 m/s)"; echo i >&3; sleep 0.5; python3 "$here/base_tracker.py" "$WINDOW"; echo k >&3; sleep 1.5
echo "--- j (counter-clockwise 1.0 rad/s)"; echo j >&3; sleep 0.5; python3 "$here/base_tracker.py" "$WINDOW"; echo k >&3; sleep 1.5
echo "--- after k (stop)"; python3 "$here/base_tracker.py" 1
echo q >&3; exec 3>&-; wait; rm -f "$fifo"
grep -E "Moving|Rotating|Stop" /tmp/simple_motion.log | sed 's/\x1b\[[0-9;]*m//g; s/.*\] //'
