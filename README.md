# ARIAC Team Workspace

## Overview
This repository provides a ROS 2 overlay workspace that layers the team
implementation (`ariac_team`) and the upstream example implementation
(`example_team`) on top of the official ARIAC 2025 base image.  The
workspace is structured so it can be cloned directly into a container or
host machine and built with `colcon` without additional path tweaks.

## Directory layout
```
.
├── docker/              # Helper scripts for containerised launches (e.g. VNC + smoke test)
├── docs/                # Reference guides, environment setup notes, design deep-dives
├── moveit_configs/      # MoveIt configuration packages shared by the workspace
├── src/
│   ├── ariac_team/      # Team ROS 2 package (nodes, launch files, configs, tests)
│   └── example_team/    # Upstream reference implementation kept for comparison
└── Dockerfile           # Overlay image that extends nistariac/ariac2025
```
Additional legacy environment setup guidance is preserved in
[`docs/environment_setup.md`](docs/environment_setup.md).

## Clone & initialise the workspace
```bash
# From inside your container home (e.g., /root)
git clone https://github.com/<your-org>/ariac_team.git
cd ariac_team
# Ensure colcon sees the ROS packages under src/
mkdir -p log build install
```
> The workspace is intended to live at `/root/ariac_team` inside your
> development container, matching the mount paths used in the Docker
> tooling included in this repo.

## Build with colcon
```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ariac_team
```
This builds only the `ariac_team` overlay package while skipping the
upstream example unless you explicitly include it in the package list.

## Source & run the entry point
After a successful build, source the workspace and start the coordinator
node that wires up the MVP smoke pipeline:
```bash
source install/setup.bash
ros2 run ariac_team mvp_coordinator
```
The entry point loads the default
[`config/ariac_team_config.yaml`](src/ariac_team/config/ariac_team_config.yaml)
parameters and can be overridden at launch time (see the next section).

## Selecting team & trial YAML files
The package ships with curated configuration files located under
[`src/ariac_team/config`](src/ariac_team/config):
- `ariac_team_config.yaml`: default sensor, motion, and competition
  toggles used by the coordinator.
- `config/trials/*.yaml`: targeted trials such as `mvp_smoke.yaml`,
  `conveyor_to_tester.yaml`, `tester_to_agv.yaml`, and
  `shipping_submit.yaml` for staged testing.

When launching, provide the desired files via launch arguments, e.g.:
```bash
ros2 launch ariac_team mvp_smoke.launch.py \
  team_config:=/team_ws/src/ariac_team/config/ariac_team_config.yaml \
  trial_config:=/team_ws/src/ariac_team/config/trials/mvp_smoke.yaml
```
The same paths are used inside containers because the repository is
mounted at `/team_ws/src` by the provided Docker tooling.

## Launching the full stack
Detailed launch instructions (Gazebo simulation, world creation, control
stack startup, and smoke tests) are documented in
[`docs/launch_guide.md`](docs/launch_guide.md).  Key entry points are:
- [`launch/mvp_smoke.launch.py`](src/ariac_team/launch/mvp_smoke.launch.py):
  standard ROS 2 launch file for the MVP pipeline.
- [`docker/start_mvp.sh`](docker/start_mvp.sh): wraps VNC + launch for a
  one-command smoke test inside the container.
- [`docker/start_.sh`](docker/start_.sh): reference script for launching
  the full ARIAC simulation and app stack manually.

## Further reading
To dive deeper into module-level design decisions and extension points:
- [`docs/design_notes.md`](docs/design_notes.md): outlines the pipeline,
  robot coordination, and environment abstraction layers with pointers
  to the relevant Python modules.
- [`docs/environment_setup.md`](docs/environment_setup.md): legacy notes
  covering the broader ARIAC competition background and baseline setup.

Community contributions should follow the existing ROS 2 coding style
(`rclpy`, launch argument usage, YAML parameterisation).  See
[`docs/launch_guide.md`](docs/launch_guide.md) before filing issues to
ensure you have reproduced the canonical launch workflow locally.
