#!/bin/bash
# Start the full stack and check the UAV is healthy (hovering near the origin,
# simulator near real time); retry up to 3 times. usage: stack_verified.sh [drain_s]
# env: SAGE_WORLD (default sage_test)
S=/tmp/sage_logs
source /opt/ros/humble/setup.bash; source ~/sage_ws/install/setup.bash
for attempt in 1 2 3; do
  ~/SAGE-UAV/scripts/killall.sh >/dev/null 2>&1; sleep 3
  # Let the machine settle: booting the simulator under load causes PX4
  # preflight failures (High Gyro Bias / attitude failure).
  for i in $(seq 1 45); do
    load=$(cut -d' ' -f1 /proc/loadavg)
    awk -v l=$load 'BEGIN{exit !(l<2.0)}' && break; sleep 2
  done
  echo "attempt $attempt: load before start = $(cut -d' ' -f1 /proc/loadavg)"
  : > $S/stack.out
  nohup ~/SAGE-UAV/scripts/stack.sh ${1:-1200} > $S/stack.out 2>&1 &
  for i in $(seq 1 60); do grep -qE "stack-up|STARTUP_TIMEOUT|PREFLIGHT_NOT_READY" $S/stack.out 2>/dev/null && break; sleep 5; done
  if ! grep -q "stack-up" $S/stack.out; then echo "attempt $attempt: stack did not come up: $(grep -E 'STARTUP_TIMEOUT|PREFLIGHT_NOT_READY' $S/stack.out | head -1)"; continue; fi
  sleep 35
  pos=$(timeout 6 ros2 topic echo /fmu/out/vehicle_local_position --once 2>/dev/null | grep -E "^(x|y|z):" | awk '{print $2}' | tr '\n' ' ')
  rtf=$(timeout 8 gz topic -e -t /stats -n 1 2>&1 | tr -d '\n ' | grep -o "real_time_factor:[0-9.]*" | cut -d: -f2)
  ok=$(echo "$pos $rtf" | awk '{ if (NF>=4 && ($1*$1+$2*$2)<16 && $3<-0.8 && $3>-3.5 && $4>0.5) print "yes"; else print "no" }')
  echo "attempt $attempt: pos=[$pos] rtf=$rtf healthy=$ok"
  [ "$ok" = "yes" ] && { echo "STACK_HEALTHY"; exit 0; }
done
echo "STACK_FAILED"; exit 1
