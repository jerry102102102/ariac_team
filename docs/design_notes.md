# Design Notes

This document summarises the structure of the `ariac_team` package and
points to the modules where specific behaviours are implemented.  Use it
as a jumping-off point before diving into the codebase.

## Pipeline architecture
- [`pipeline/__init__.py`](../src/ariac_team/ariac_team/pipeline/__init__.py)
  exports convenience functions for constructing stage sequences.
- [`pipeline/base.py`](../src/ariac_team/ariac_team/pipeline/base.py)
  defines the abstract `Stage` class and common utilities shared by all
  stages.
- [`pipeline/stages.py`](../src/ariac_team/ariac_team/pipeline/stages.py)
  houses the concrete MVP stages (conveyor pickup, tester handling, AGV
  submission).  Each stage focuses on a single task and communicates via
  the coordinator's shared context object.

## Robot coordination
- [`robots.py`](../src/ariac_team/ariac_team/robots.py) wraps access to
  the ARIAC-provided robot interfaces and MoveIt motion planning helpers.
  This is the main entry point for commanding UR arm trajectories.
- [`nodes/mvp_coordinator.py`](../src/ariac_team/ariac_team/nodes/mvp_coordinator.py)
  orchestrates the pipeline stages, monitors competition state, and
  coordinates robot actions.  Extend this node when introducing new
  behaviours or handling competition events.

## Environment abstraction
- [`environment.py`](../src/ariac_team/ariac_team/environment.py) models
  sensors, breakbeams, and world entities exposed by ARIAC.  It provides
  convenience functions for tracking AGVs, trays, and conveyor cells.
- Configuration is handled through
  [`config/ariac_team_config.yaml`](../src/ariac_team/config/ariac_team_config.yaml)
  and the trial definitions in
  [`config/trials`](../src/ariac_team/config/trials).

## Launch files & scripts
- [`launch/mvp_smoke.launch.py`](../src/ariac_team/launch/mvp_smoke.launch.py)
  is the canonical launch for the MVP pipeline.  Use the `team_config`
  and `trial_config` arguments to point at alternative YAML files.
- [`docker/start_mvp.sh`](../docker/start_mvp.sh) spins up VNC, sources the
  workspace, and runs the smoke launch in one command.
- [`docker/start_.sh`](../docker/start_.sh) is a lower-level script that
  launches Gazebo and the ARIAC app for manual experimentation.

## Testing
- [`tests`](../src/ariac_team/tests) contains pytest scaffolding for unit
  tests.  Extend this directory as new modules are added.

For historical environment context, see
[`docs/environment_setup.md`](environment_setup.md).  Implementation
changes should be reflected both in code comments and in this document so
contributors can keep track of the evolving architecture.
