# Design: active identity check for possibly-moving people

Status: DESIGN ONLY (2026-09-25), nothing implemented. Problem: in sage_hard the walker
W2 (path (-5,5)-(-5,11)) is reported twice in ~half of the runs (FP). Static S3 (3.5 m from
the W1 path) and S2 (4.0 m from the W2 path) must not be merged into walkers.

## What the logs say (all sage_hard world.logs 0925_02..04, raw localizations)
| estimator | static people | walkers |
|---|---|---|
| 2 s slope (current planner) | p50 0.29, p90 1.04, p97 2.05 m/s | p50 0.58, p90 1.89 |
| 8 s least-squares slope | p90 0.18, p97 0.23 | p50 0.15, p75 0.32, p90 0.45 |
| 8 s slope > 0.30 m/s | 0 % flagged moving | only 26 % of windows flagged |
| world-model MOVING label | 20-39 % of updates | 41-65 % of updates |
- Longer windows make "moving" a HIGH-SPECIFICITY test (<=3 % of static windows exceed
  0.25 m/s at 8 s) but a LOW-SENSITIVITY one (~30-40 % of walker windows). A low slope
  can NOT prove a person is static.
- Why walkers hide: they turn around every ~10 s sim time (6 m legs at 0.6 m/s, RTF < 1 makes
  it slower in wall time), so most 8 s windows contain a reversal and the net slope cancels.
  The test world has unusually short walker paths; real walkers rarely reverse that often.
- Distance rule cannot work: W2 duplicates were 2.7-6 m apart; S3/S2 sit 3.5-4 m from walker
  paths. A 3.2 m merge radius lost S3 (trial 0925_040416). Flagging by reachability is noise
  (verifications are tens of seconds apart, reach always hits the 8 m cap).

## Principle
Never delete a possible person on uncertainty. Only merge on POSITIVE evidence that the
earlier place is empty. Everything unresolved stays in the report (recall first).

## Design
Phase `identity_check`, runs after the last coverage waypoint, before `complete_mission`
(skipped for integer quantity missions that already completed).

1. Ambiguous set A: verified entries that have another verified entry within 8 m
   (beyond the 2.5 m duplicate radius). Speed flags are NOT used (unreliable). In sage_hard
   nearly all 5 people qualify.
2. Route: nearest-neighbour order from the UAV, each stop energy-checked with the existing
   `estimated_cost` (skip and mark UNKNOWN if not affordable). Reuse
   `calculate_reachable_viewpoint`, `viewpoint_reached`, `publish_viewpoint`, the obstacle map.
3. Check at entry E=(x,y): viewpoint 3.5 m from E, yaw toward E, altitude as now. Hover and
   dwell up to `identity_dwell_s` = 8 s, collecting fresh localized positions (confidence
   >= 0.5, bbox ok) that lie within 1.5 m of E. Only count frames where E is in view
   (UAV within 0.5 m of the viewpoint, yaw error small, range <= 6 m).
4. Outcome per entry:
   - STATIC_PRESENT: >= 3 detections within 1.5 m of E spread over >= 4 s AND their extent
     (90th-percentile distance from the median) <= 1.2 m. (static r90: p90 1.02, p97 1.30 -> tune)
   - MOVER_HERE: detections exist but drift (extent > 1.5 m).
   - ABSENT: zero qualifying detections while E was in view for the whole dwell. Before
     concluding ABSENT re-check once from the opposite side (+~15 s) so a single occluded
     or missed view cannot cause a merge.
   - UNKNOWN: not reached / not in view / energy / time budget. Treated as PRESENT for merging
     (i.e. never merge).
5. Resolution (pure function `resolve_identities(entries, outcome)`, unit-testable):
   entries E and F within 8 m are the SAME person iff neither is STATIC_PRESENT and neither
   is UNKNOWN (both ABSENT/MOVER_HERE). One STATIC_PRESENT entry => it is a static person
   and stays separate from everything beyond 2.5 m. Merged output keeps the higher-confidence
   position and lists `also_seen_at` for the other.
6. Report: `identity_checks` = [{id, outcome, merged_into}], counts of merged/unknown; the log
   line prints the same. The scorer is unchanged (it reads the merged list).
7. Time budget: `identity_budget_s` = 120 s hard cap for the phase. It must be allowed to run
   past `mission_timeout_s` (now 900 s; sage_hard runs already end at 707-900 s) - raise the
   timeout to ~1100 s or run the phase from the timeout path too.

## Known weaknesses (be honest in any writeup)
- A walker passing the checked spot during the dwell looks present; the persistence /
  extent rule (step 4) mitigates but 8 s of a slow walker can still look static (walker
  8 s extent p50 0.89 m). Mitigation candidates: longer dwell, two dwells at different times.
- False ABSENT (detector miss) + a mover nearby => a real person merged. Rare with
  the two-sided re-check, but nonzero; quantify with N trials.
- Cost: ~15-20 s per checked entry, 60-100 s per mission on sage_hard.
- Both walkers absent and <8 m apart would be merged (not the case in sage_hard: paths >10 m apart).
- Perception + localization still use tuned camera delay and SITL ground truth where noted
  in progress.md.

## Validation plan
1. Unit tests for `resolve_identities` (W2 pair both absent -> merged; S3 present + W1 absent
   -> distinct; UNKNOWN -> never merged; chains).
2. Replay: extent/persistence thresholds against the world.log windows (done for slope; extent
   thresholds need the dwell data of a live run).
3. Live: 5 sage_hard trials, compare FP/FN/duration against the 2.5 m baseline
   (20/20 recall, 2 FP in 4 scored runs). Extra worlds: a static person 3 m from a walker path;
   a longer walker path (12-16 m) to test the sim-artifact hypothesis.

## Cheaper alternatives (if the check is too costly)
- Lengthen walker paths in the world so that motion is visible in 8 s windows, then use an
  8 s evidence window (moving = slope > 0.3, <=3 % false moving on statics).
- Verify with a longer dwell only for ambiguous entries and use the 8 s slope as one vote.
