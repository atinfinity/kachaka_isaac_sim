# ROS 2 side of the simulator: Nav2, slam_toolbox, RViz and teleop.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-rviz2 \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Virtual display: lets RViz run headless (used by scripts/record_demo.sh).
RUN apt-get update && apt-get install -y --no-install-recommends xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
# kachaka_description (meshes for RViz) and our bringup package.
COPY third_party/kachaka-api/ros2/kachaka_description src/kachaka_description
COPY ros2_ws/src src
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install --event-handlers console_direct- \
        --packages-select kachaka_description kachaka_sim_bringup

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
