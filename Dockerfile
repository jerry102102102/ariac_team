# ================================================================
# Team overlay image extending ARIAC base
# ================================================================

FROM nistariac/ariac2025:latest
# GUI 需要的 minimal 套件
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      xvfb x11vnc fluxbox xterm \
    && rm -rf /var/lib/apt/lists/*
    
# Create a new overlay workspace
ENV TEAM_WS=/team_ws
RUN mkdir -p $TEAM_WS/src
WORKDIR $TEAM_WS

# Copy the ROS 2 packages and supporting assets into the overlay workspace
COPY ./src/ $TEAM_WS/src/
COPY ./moveit_configs/ $TEAM_WS/moveit_configs/
COPY ./docs/ $TEAM_WS/docs/
COPY ./docker/ $TEAM_WS/docker/

# Update apt and install any OS dependencies needed for rosdep
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# RUN pip3 install -r $TEAM_WS/src/requirements.txt --break-system-packages

# Build the workspace (symlink install is fine for overlays)
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
                  source /ariac_ws/install/setup.bash && \
                  colcon build --symlink-install"

# Source automatically in container
RUN echo "source /team_ws/install/setup.bash" >> /root/.bashrc
COPY docker/start_vnc.sh /usr/local/bin/start_vnc.sh
RUN chmod +x /usr/local/bin/start_vnc.sh

WORKDIR $TEAM_WS
CMD ["bash"]
