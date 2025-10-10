# Launch & Test Guide

This document provides the end-to-end commands required to spin up the
ARIAC simulation, create a competition world, launch the team control
stack, and run the included smoke tests.  The instructions assume the
repository has been cloned to `/root/ariac_team` inside a development
container and that you are using the Docker tooling bundled with this
workspace.

## 1. Build the overlay image (optional but recommended)
If you plan to run everything inside a container that already includes
the workspace, build the overlay image once:
```bash
cd /root/ariac_team
docker build -t ariac_team:latest .
```
This stage installs any ROS dependencies via `rosdep` and compiles the
workspace inside the image, so runtime containers only need to source the
pre-built install space.

## 2. Start a development container
Use Docker Compose to launch a container with VNC, Gazebo, and ROS 2
pre-configured:
```bash
cd /root/ariac_team
docker compose up ariac      # or `ariac_nvidia` on GPU hosts
```
The Compose file mounts your local `src/` directory into `/team_ws/src`
inside the container so any changes are reflected immediately.

### Alternate: direct image run
If you built the overlay image, you can also start it directly:
```bash
docker run --rm -it \
  --name ariac_team_dev \
  --gpus all \
  -p 8080:8080 -p 5900:5900 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /root/ariac_team/src:/team_ws/src:rw \
  -v /root/ariac_team/moveit_configs:/team_ws/moveit_configs:rw \
  -v /root/ariac_team/docs:/team_ws/docs:rw \
  ariac_team:latest bash
```
Adjust the `--gpus` flag if you are on a CPU-only host.

## 3. Initialise the ROS 2 environment inside the container
Once inside the container shell (either via `docker compose exec` or a
terminal opened by VNC):
```bash
source /opt/ros/jazzy/setup.bash
source /ariac_ws/install/setup.bash   # Provided by the base image
colcon build --packages-select ariac_team
source install/setup.bash
```
> The initial `colcon build` is only required if you are developing
> locally.  The `docker build` workflow already compiles these packages.

## 4. Launch the ARIAC simulation world
Bring up the Gazebo simulation with the desired team and trial
configuration:
```bash
ros2 launch ariac_gz ariac.launch.py \
  user_config:=/team_ws/src/example_team/config/example_team_config.yaml \
  trial_config:=/team_ws/src/example_team/config/trials/LHAF9835.yaml
```
You can swap in your own trial file from
`src/ariac_team/config/trials/` after copying or mounting it to the
container.

## 5. Run the team control stack
In a second terminal (still inside the container), source the overlay and
launch the smoke pipeline:
```bash
source /team_ws/install/setup.bash
ros2 launch ariac_team mvp_smoke.launch.py \
  team_config:=/team_ws/src/ariac_team/config/ariac_team_config.yaml \
  trial_config:=/team_ws/src/ariac_team/config/trials/mvp_smoke.yaml
```
The launch file wires the coordinator node together with its dependencies
and applies the selected configuration files.

## 6. One-command smoke test
For a quick end-to-end validation, execute the bundled script which
starts a headless VNC session and launches the MVP smoke test:
```bash
./docker/start_mvp.sh
```
This script assumes it is run from inside the container and will spawn a
VNC server on port 5900 for optional visual inspection.

## 7. Stopping the stack
- To stop the smoke test, interrupt the launch process with `Ctrl+C`.
- To stop Gazebo, interrupt the `ariac_gz` launch.
- To tear down the container, press `Ctrl+C` in the Compose terminal or
  run `docker compose down`.

Refer to [`docs/design_notes.md`](docs/design_notes.md) for details on the
pipeline stages and how to extend them for competition scenarios beyond
the MVP smoke test.
