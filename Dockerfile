# ================================================================
# Team overlay image extending ARIAC base
# ================================================================

FROM ariac:latest

# Create a new overlay workspace
ENV TEAM_WS=/team_ws
RUN mkdir -p $TEAM_WS/src
WORKDIR $TEAM_WS

# Copy team packages into the overlay workspace
COPY ./ $TEAM_WS/src/

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

WORKDIR $TEAM_WS
CMD ["bash"]
