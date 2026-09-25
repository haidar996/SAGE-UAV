#!/bin/bash
cp ~/SAGE-UAV/scripts/run_trials.sh /tmp/rt.sh
export SAGE_RECORD=1
exec bash /tmp/rt.sh "$@"
