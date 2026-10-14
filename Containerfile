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
