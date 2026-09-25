# Evaluation

## Protocol
1. Start the stack with the health gate (`scripts/stack_verified.sh`), then the mission nodes, then send the mission
   `Find all people in this area and report their locations`.
2. Wait for `MISSION COMPLETE` (planner log). The mission ends by area coverage, timeout (1200 s) or UAV loss.
3. Save every log to `results/logs/<run id>/` and score the planner log against the world's ground truth
   (`scripts/score_trial.py`): true positives, false reports, misses, mean and maximum position error, duration, rejected candidates.
4. Append the row to `results/trials_<world>.csv`. Skipped or failed runs are recorded too.

`scripts/run_trials.sh N world` performs steps 1-4 for N runs.

## Metrics
| metric | definition |
|---|---|
| recall | matched true people / true people, summed over runs |
| precision | matched reports / all reports |
| position error | distance between a report and its matched person (or path) |
| mission time | planner-reported duration from acceptance to completion |

## Results (simulation)
| world | runs | recall | precision | mean error | mission time, median (range) |
|---|---|---|---|---|---|
| `sage_sar` | 5 | 100% (15/15) | 100% | 0.30 m | 242 s (199-311) |
| `sage_rescue` | 7 | 89% (25/28) | 89% | 0.46 m | 630 s (518-1200) |
| `sage_hard` | 4 | 100% (20/20) | 91% | 0.43 m | 895 s (707-900) |

Per-run tables and notes: [`results.md`](results.md); machine-readable rows: `results/trials_*.csv`; figures: `results/figures/`.

## Reproducing the figures
```bash
python3 scripts/make_figures.py       # results/figures/*.png and results/summary.md from results/trials_*.csv
```
