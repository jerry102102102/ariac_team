#!/usr/bin/env bash
set -e

# 顯示編號可以改成 :99 也行
export DISPLAY=:1
export QT_X11_NO_MITSHM=1
# 若之後想用 offscreen 渲染，改用： export QT_QPA_PLATFORM=offscreen

# 啟 Xvfb (虛擬顯示)
Xvfb ${DISPLAY} -screen 0 1920x1080x24 &

# 輕量視窗管理器（避免某些 Qt/GL 想要 WM）
fluxbox >/tmp/fluxbox.log 2>&1 &

# 開 VNC 伺服器（5900）
x11vnc -display ${DISPLAY} -forever -shared -nopw -rfbport 5900 > /tmp/x11vnc.log 2>&1 &

# 留在互動 shell（你可以在容器內手動啟 sim）
exec bash

