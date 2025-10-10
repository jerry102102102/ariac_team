# 1) 啟 Xvfb（若已在跑會報錯，可忽略）
export DISPLAY=:1
Xvfb :1 -screen 0 1920x1080x24 &

# 2) （可選）視窗管理器
fluxbox >/tmp/fluxbox.log 2>&1 &

# 3) 開無密碼的 x11vnc（不要帶 -rfbauth）
x11vnc -display :1 -forever -shared -nopw -rfbport 5900 &
# 4) 啟 sim
source /opt/ros/jazzy/setup.bash
source /ariac_ws/install/setup.bash
source /team_ws/install/setup.bash
ros2 launch ariac_gz ariac.launch.py \
  user_config:=/team_ws/src/example_team/config/example_team_config.yaml \
  trial_config:=/team_ws/src/example_team/config/trials/LHAF9835.yaml

# 5) 啟 app
ros2 run ariac_app app