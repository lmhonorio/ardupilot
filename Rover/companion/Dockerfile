FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONUNBUFFERED=1
RUN apt-get -o Acquire::Retries=3 update && apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
    build-essential cmake git python3-dev python3-venv python3-pip python3-colcon-common-extensions \
    python3-numpy python3-scipy python3-opencv python3-yaml python3-matplotlib \
    ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-cv-bridge \
    ros-jazzy-sensor-msgs-py ros-jazzy-message-filters ros-jazzy-tf-transformations \
    libpcl-dev libeigen3-dev libapr1-dev ros-jazzy-ament-cmake-auto \
    tini iproute2 iputils-ping procps && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/rover/Rover
COPY companion/requirements.txt /tmp/companion-requirements.txt
RUN python3 -m venv --system-site-packages /opt/companion-venv && \
    /opt/companion-venv/bin/pip install --no-cache-dir -r /tmp/companion-requirements.txt && \
    /opt/companion-venv/bin/pip install --no-cache-dir --no-deps MAVProxy==1.8.75
# Include the real MID360 driver as well as its ROS interfaces in the same image.
RUN git init /tmp/Livox-SDK2 && cd /tmp/Livox-SDK2 && \
    git remote add origin https://github.com/Livox-SDK/Livox-SDK2.git && \
    git fetch --depth 1 origin c0796f04c143143899c87a773d9f6b7136453c0b && git checkout FETCH_HEAD && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j1 && \
    cmake --install build && ldconfig && rm -rf /tmp/Livox-SDK2
RUN git init ros2/src/livox_ros_driver2 && cd ros2/src/livox_ros_driver2 && \
    git remote add origin https://github.com/Livox-SDK/livox_ros_driver2.git && \
    git fetch --depth 1 origin 21445540f0d100dc86a7e6df312dd70bbdb4afdf && git checkout FETCH_HEAD && \
    cp package_ROS2.xml package.xml && cp -r launch_ROS2 launch && rm -rf .git
COPY ros2/src/rover_mission_interfaces ros2/src/rover_mission_interfaces
COPY ros2/deps/FAST_LIO ros2/deps/FAST_LIO
COPY python/simulation/ros2/patch_fast_lio.py /tmp/patch_fast_lio.py
RUN python3 /tmp/patch_fast_lio.py ros2/deps/FAST_LIO && \
    source /opt/ros/jazzy/setup.bash && cd ros2 && \
    MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --merge-install --executor sequential \
      --base-paths src/livox_ros_driver2 src/rover_mission_interfaces deps/FAST_LIO \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy \
      -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 && \
    rm -rf build log
COPY ros2/src/rover_companion_bridge ros2/src/rover_companion_bridge
RUN source /opt/ros/jazzy/setup.bash && source ros2/install/setup.bash && cd ros2 && \
    MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --merge-install --executor sequential \
      --base-paths src/rover_companion_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf build log
COPY python python
COPY estacoes_recarga.txt estacoes_recarga.txt
COPY companion companion
# Upstream opens these debug streams even with runtime_pos_log_enable=false.
# Keep valid handles without growing debug files in the container layer.
RUN mkdir -p python/run ros2/run ros2/deps/FAST_LIO/Log ros2/deps/FAST_LIO/PCD /data /missions /config && \
    for log in mat_pre.txt mat_out.txt dbg.txt pos_log.txt; do \
      ln -s /dev/null "ros2/deps/FAST_LIO/Log/$log"; done && \
    chmod +x companion/entrypoint.sh
ENV PATH="/opt/companion-venv/bin:$PATH" PYTHONPATH=/opt/rover/Rover/python \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ENTRYPOINT ["/usr/bin/tini", "--", "/opt/rover/Rover/companion/entrypoint.sh"]
CMD ["stack"]
