# ROS 2 Jazzy + Gazebo Harmonic desktop, viewed in the browser through noVNC.
FROM docker.io/tiryoh/ros2-desktop-vnc:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher-gui \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/ubuntu/.bashrc \
 && echo '[ -f /home/ubuntu/ringquad_ws/install/setup.bash ] && source /home/ubuntu/ringquad_ws/install/setup.bash' >> /home/ubuntu/.bashrc

RUN apt-get update && apt-get install -y --no-install-recommends cmake git g++ libeigen3-dev libbullet-dev \
 && rm -rf /var/lib/apt/lists/*

# Pin and fix Bullet Featherstone's 100 rad/s default coordinate cap.
COPY hardware-blockers/physics /tmp/ringquad-physics-source
RUN bash /tmp/ringquad-physics-source/build.sh /opt/ringquad-physics \
 && rm -rf /tmp/ringquad-physics-source
ENV GZ_SIM_PHYSICS_ENGINE_PATH=/opt/ringquad-physics/lib
