#!/bin/bash
# Repeat the full mission N times in one world and score each run.
# usage: run_trials.sh [N] [world] [mission text]   (results/trials_<world>.csv, logs in results/logs/)
N=${1:-5}; export SAGE_WORLD=${2:-sage_sar}
MISSION=${3:-"Find all people in this area and report their locations"}
R=~/SAGE-UAV/results; mkdir -p $R/logs; CSV=$R/trials_$SAGE_WORLD.csv
[ -f $CSV ] || echo "trial,date,world,status,found,tp,fp,fn,mean_err_m,max_err_m,duration_s,rejected_candidates" > $CSV
source /opt/ros/humble/setup.bash; source ~/sage_ws/install/setup.bash
[ -f ~/SAGE-UAV/config/$SAGE_WORLD.env ] && source ~/SAGE-UAV/config/$SAGE_WORLD.env
DRAIN=${SAGE_DRAIN:-1200}
for t in $(seq 1 $N); do
  id=$(date +%m%d_%H%M%S)
  echo "=== trial $t/$N ($id)"
  if ! ~/SAGE-UAV/scripts/stack_verified.sh $DRAIN; then echo "$id,$(date +%F),$SAGE_WORLD,stack_failed,,,,,,,," >> $CSV; continue; fi
  ~/SAGE-UAV/scripts/start_mission_nodes.sh; sleep 10
  if [ -n "$SAGE_RECORD" ]; then   # SAGE_RECORD=1: film this trial (results/demo/<id>/demo.mp4)
    setsid nohup python3 ~/SAGE-UAV/scripts/record_demo.py --world $SAGE_WORLD --out $R/demo/$id --mission "$MISSION" > /tmp/sage_logs/recorder.log 2>&1 < /dev/null &
    sleep 3
  fi
  ros2 topic pub --once /sage/mission/command std_msgs/msg/String "{data: '$MISSION'}" >/dev/null
  for i in $(seq 1 400); do grep -q "MISSION COMPLETE" /tmp/sage_logs/planner.log && break; sleep 4; done
  sleep 25   # let it return home and land
  mkdir -p $R/logs/$id; cp /tmp/sage_logs/{planner,mission,offboard,energy,localizer,world}.log $R/logs/$id/ 2>/dev/null
  echo "$id,$(date +%F),$SAGE_WORLD,$(python3 ~/SAGE-UAV/scripts/score_trial.py /tmp/sage_logs/planner.log $SAGE_WORLD)" >> $CSV
  tail -1 $CSV
done
~/SAGE-UAV/scripts/killall.sh >/dev/null 2>&1
echo "ALL_TRIALS_DONE"
