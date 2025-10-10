#!/bin/bash
set -euo pipefail

# Launches the MVP smoke test inside the container.
export DISPLAY=:1
Xvfb :1 -screen 0 1920x1080x24 &
fluxbox >/tmp/fluxbox.log 2>&1 &
x11vnc -display :1 -forever -shared -nopw -rfbport 5900 &

source /opt/ros/jazzy/setup.bash
source /ariac_ws/install/setup.bash
source /team_ws/install/setup.bash

ros2 launch ariac_team mvp_smoke.launch.py \
  team_config:=/team_ws/src/ariac_team/config/ariac_team_config.yaml \
  trial_config:=/team_ws/src/ariac_team/config/trials/mvp_smoke.yaml
