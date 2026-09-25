# Results (simulation) - 2026-09-25

All numbers come from the trial tables in `results/trials_*.csv`, scored automatically against ground truth
(`scripts/score_trial.py`, tolerance 1.5 m; a walking person counts if the report lies within 1.5 m of its path).
Read `docs/limitations.md` before quoting any of it.

| world | runs | people found (recall) | precision | false reports | mean error | mission time, median (range) |
|---|---|---|---|---|---|---|
| `sage_sar` - 3 standing people | 5 | 100% (15/15) | 100% | 0 | 0.30 m | 242 s (199-311 s) |
| `sage_hard` - 3 standing + 2 walking people, 6 obstacles | 4 | 100% (20/20) | 91% | 2 | 0.43 m | 895 s (707-900 s) |
| `sage_rescue` - 3 standing + 1 walking, rich scene | 6 | 88% (21/24) | 88% | 3 | 0.46 m | 630 s (518-1200 s) |

(`sage_sar`: the 5 runs of the final configuration; an earlier 5-run baseline found 14/15 with 0 false reports.
`sage_hard`: the 4 scored runs of the final merge configuration; other runs were skipped or cut off, see progress.md.)

## `sage_rescue` runs

| run | found | correct | false | missed | mean error | time | note |
|---|---|---|---|---|---|---|---|
| 0925_062308 | 3 | 2 | 1 | 2 | 0.66 m | 630 s | walker never verified; one standing person 1.6 m off (outside the 1.5 m tolerance) |
| 0925_063805 | 5 | 4 | 1 | 0 | 0.34 m | 570 s | walker reported twice |
| 0925_065952 | 5 | 4 | 1 | 0 | 0.13 m | 518 s | walker reported twice |
| 0925_071213 | 4 | 4 | 0 | 0 | 0.31 m | 1200 s | all found, no duplicate; hit the 1200 s mission timeout (sweep 5/16 for a long time); this run is the demo video |
| 0925_073827 | 3 | 3 | 0 | 1 | 0.76 m | 827 s | walker missed |
| 0925_141115 | 4 | 4 | 0 | 0 | 0.57 m | 611 s | all found, no duplicate, completed normally: THE DEMO FLIGHT (both videos) |

## What the numbers say
- Standing people: found in 14 of 15 rescue attempts (the miss is a 1.6 m localization error), 15 of 15 in `sage_sar`.
  Localization error is 0.13-0.4 m in good runs.
- The single walking person is the weak point: found in 4 of 5 runs, but reported twice in 3 of them, and it consumes
  most of the candidate-handling time (rejected candidates 6-17 per run).
- Time: 8.6-14 min for a 24 x 24 m area; a run that hits the 1200 s timeout still reports everything verified so far.
- Startup reliability: about 37% of stack start attempts fail the health gate (mostly real-time factor < 0.5) and
  about 7% show a startup flip; the runner retries up to 5 times and logs every attempt.

## What was tried and did NOT work (kept on purpose)
- Merging duplicates by distance (3.2 m) lost a real person; an early candidate drop lost two; speed/motion-state
  cannot separate walkers from standing people (`docs/solutions_plan.md`, `docs/design_identity_check.md`).
- Longer walker paths made motion more visible (walker windows flagged moving 32% -> 53%) but did not remove duplicates.

## Not done
- Active identity check (design + tested decision logic exist, not wired in), Experiments A-F from `docs/steps.md`
  (no controlled comparison was run), real-hardware tests, Claude parser live test (no API key).

## Excluded runs (disclosed)
Two further `sage_rescue` flights (0925_133529, 0925_135407) ended as `uav_lost`: the drone flew away (about 100 m) after the position
stream froze for several seconds. Both had a heavier overhead Gazebo camera (800x800 and 512x512 pixels) in the world; the five earlier
rescue flights without it completed normally. The overhead camera was cut to 384x384 at 2 Hz and the next flight (the demo flight)
completed normally. One unexplained 17 s stall + flyaway had already happened without any overhead camera (0925_024917), so the
camera is a suspected contributor, not a proven cause. These two runs are not in the table above.

## Demo videos (same flight, 0925_141115, 2x real-time speed, 5.3 min each)
- `demo/sage_uav_camera_view.mp4`: onboard camera with YOLO boxes, live mission map, stage bar, battery.
- `demo/sage_uav_gazebo_overhead.mp4`: fixed overhead Gazebo camera (30 m up) with the drone trail, verified people and the same HUD.
- Rendered offline from a light in-flight log (`scripts/record_raw.py` + `scripts/render_demo.py`), so recording did not load the simulator.
  The overhead render only updates about once per second (the sim was already near its limit); the drone marker and trail are smooth.
