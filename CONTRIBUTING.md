# Contributing

Issues and pull requests are welcome.

- **Bugs:** include the world, the command you sent, the run folder under `results/logs/<id>/`, and the PX4 and Gazebo versions
  (`environment/README.md`).
- **Code:** keep nodes parameterised (no magic numbers in logic), keep pure decision logic importable without ROS, and add a unit test in
  `src/sage_px4_interface/test/` for any new pure function.
- **Changes to behaviour:** run `scripts/run_trials.sh 5 sage_rescue` before and after and include the summary rows in the pull request.
- **Style:** PEP 8, docstrings for public functions, no committed secrets (`ANTHROPIC_API_KEY` is read from the environment only).
