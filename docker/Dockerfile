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

# The simulator's kachaka-api gRPC server and the official client, for
# scripts/fake_sim.py and tests/api_check.py in CI. kachaka-api asks for
# numpy>=2.2 only for its vision and geometry helpers, which neither uses, so
# install it without dependencies and keep ROS's numpy.
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip python3-pil \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir --break-system-packages --no-deps \
        kachaka-api==3.17.5.0 grpcio==1.66.1 "protobuf>=5.27.2,<6"

WORKDIR /ws
# kachaka_description (meshes for RViz), kachaka_interfaces (command action,
# locations) and our bringup package.
COPY third_party/kachaka-api/ros2/kachaka_description src/kachaka_description
COPY third_party/kachaka-api/ros2/kachaka_interfaces src/kachaka_interfaces
COPY ros2_ws/src src
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install --event-handlers console_direct- \
        --packages-select kachaka_description kachaka_interfaces kachaka_sim_bringup

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
