# Solutions plan for the remaining medium/high problems (2026-09-25)

Everything is pushed at the end of the project (user decision); until then a local git bundle
lives in ~/backups. Low-priority items (LLM parser key, energy-model realism, world-model motion
label) are intentionally parked.

## P1 - Mission time / lost coverage (HIGH)
Facts: 88 rejected candidates in 8 hard runs = ~256 s/run (28 %). 27 were ghost tracks refined onto an
already verified person, 42 true phantoms, 19 real unverified people. Real verifications take up to 20 s
after the UAV is within 6 m (p50 8, p90 20), so the 25 s candidate timeout cannot be shortened safely; a
"no match by X s" early abort cut 25/41 real candidates in the log data (the status log is throttled, so the
data cannot support it).
Done: candidates refined within 2.0 m of a verified person are dropped at once (equivalent to the merge that
verification would do, no recall change); mission_timeout_s 900 -> 1200 (commit 9af45a2). Validation batch
of 3 hard trials running when this was written.
Next if still slow: log per-tick evidence (sufficient/match/evidence_s) to design a safe early abort from the
real verification signal; reduce phantoms at the source (YOLO detections on distractors: require the
localized position to be consistent over >= 3 frames before a track can become a candidate).

## P2 - Sim stall + flyaway + deadlock (HIGH)
Facts: run 0925_024917 shows a 17 s stall (battery, voltage, camera detections all frozen) at t = 393 s, then
the UAV was 24 m from home and finally sat on the ground at 26.7 m; the Mission Manager then rejected every
viewpoint ("too far from UAV") 2133 times = deadlock. One stall in 8 runs; no OOM in the kernel log; cause not
proven. PX4 behaviour (docs): offboard setpoints slower than 2 Hz for COM_OF_LOSS_T (default 1 s) make PX4
leave offboard and run the COM_OBL_RC_ACT failsafe.
Mitigations (independent of the unproven cause):
1. UAV-lost guard in the planner (written, uncommitted/unbuilt at the time of writing): outside area+6 m or
   on the ground for 8 s during a mission -> mission ends with the partial report (`uav_lost`), so no deadlock
   and the trial is scored instead of skipped.
2. Network hardening: ROS_LOCALHOST_ONLY=1 (+ GZ_IP=127.0.0.1) in stack.sh so Wi-Fi changes cannot disturb
   discovery (the kernel log shows a Wi-Fi reassociation about 2 min after the freeze; precaution only).
3. PX4 offboard-loss policy: COM_OBL_RC_ACT=4 (Land) and COM_OF_LOSS_T=3 s, so a stall ends with a landing in
   place, not a wander. To be tested by pausing the offboard node (SIGSTOP 10 s) during a mission - this is also
   a Step 22 robustness test.

## P3 - Walker duplicates (MEDIUM)
Facts: identity across gaps of tens of seconds cannot come from speed (windowed slope), the world-model motion
label, or distance (S3 is 3.5 m from the W1 path; W2 duplicates were 2.7-6 m apart). Literature on person
re-identification for UAV tracking relies on appearance (deep ReID, skeleton); here every person is the same actor
mesh, so appearance is uninformative. Search theory offers "negative information": a place looked at and found
empty is evidence that the earlier person left.
New evidence (scripts/redetect_evidence.py): a static person keeps being re-detected at its verified spot
>= 15 s later; walkers do less. Share of verified entries with >= k late re-detections within 1.5 m:
k=5: static 86 % vs walker 32 %; k=8: static 59 % vs walker 11 %; k=12: 41 % vs 0 %. So >= 8 means
"confirmed static" with ~87 % precision, but 41 % of static entries never reach it, so it cannot drive merging
alone without dropping real people.
Plan: (a) use it to skip revisits for confirmed-static entries in the active identity check
(docs/design_identity_check.md) - the check then covers only unconfirmed entries that have a neighbour
within 8 m, roughly halving its cost; (b) merge only on positive absence evidence (both ends ABSENT/MOVER at
the revisit), never on speed or distance. Free negative information (a verified spot inside the camera view
with no detection) is a cheaper second source once the localizer exposes the camera model to the planner.

## P4 - Reproducibility and honesty (MEDIUM)
docs/limitations.md lists all simulation-only shortcuts. Trials: >= 5 per configuration on an idle machine,
health gate + retries stay; report skipped/lost runs explicitly in the tables.

## Order of work
1. Validate P1 (batch running). 2. Build + test P2 (guard, localhost, offboard-loss policy, SIGSTOP test).
3. Implement P3 (identity check) with unit tests, then 5 hard trials. 4. Steps 22-25.
