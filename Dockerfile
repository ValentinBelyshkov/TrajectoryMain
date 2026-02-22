# Image taken from https://github.com/turlucode/ros-docker-gui
# For Jetson Orin (ARM64), use the appropriate base image
# If building on x86_64 for ARM64 cross-compilation, use multi-arch or QEMU
ARG BUILDPLATFORM=linux/amd64
ARG TARGETPLATFORM=linux/amd64
FROM --platform=$BUILDPLATFORM osrf/ros:humble-desktop-full-jammy

# Detect architecture
RUN echo "Building for platform: $TARGETPLATFORM" && \
    if [ "$(uname -m)" = "aarch64" ]; then \
      echo "Building for ARM64 (Jetson Orin)"; \
    else \
      echo "Building for x86_64 (amd64)"; \
    fi

RUN apt-get update

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y gnupg2 curl lsb-core vim wget python3-pip libpng16-16 libjpeg-turbo8 libtiff5

RUN apt-get install -y \
    # Base tools
    cmake \
    build-essential \
    git \
    unzip \
    pkg-config \
    python3-dev \
    # OpenCV dependencies
    python3-numpy \
    # Pangolin dependencies
    libgl1-mesa-dev \
    libglew-dev \
    libpython3-dev \
    libeigen3-dev \
    apt-transport-https \
    ca-certificates\
    flask \
    pymap3d \
    software-properties-common

RUN apt update

# Build OpenCV
# Set ARM64-specific CMake flags for Jetson Orin
ARG OPENCV_CMAKE_FLAGS="-D CMAKE_BUILD_TYPE=Release -D BUILD_EXAMPLES=OFF -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local"
RUN if [ "$(uname -m)" = "aarch64" ]; then \
      export OPENCV_CMAKE_FLAGS="$OPENCV_CMAKE_FLAGS -D CMAKE_C_FLAGS=-march=armv8-a -D CMAKE_CXX_FLAGS=-march=armv8-a"; \
    fi

RUN apt-get install -y python3-dev python3-numpy python2-dev
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev
RUN apt-get install -y libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
RUN apt-get install -y libgtk-3-dev

RUN cd /tmp && git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    git checkout 4.4.0 && mkdir build && cd build && \
    cmake $OPENCV_CMAKE_FLAGS .. && \
    make -j8 && make install && \
    cd / && rm -rf /tmp/opencv

# Build Pangolin
# Set ARM64-specific CMake flags for Jetson Orin
ARG PANGOLIN_CMAKE_FLAGS="-DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14 -DCMAKE_INSTALL_PREFIX=/usr/local"
RUN if [ "$(uname -m)" = "aarch64" ]; then \
      export PANGOLIN_CMAKE_FLAGS="$PANGOLIN_CMAKE_FLAGS -DCMAKE_C_FLAGS=-march=armv8-a -DCMAKE_CXX_FLAGS='-march=armv8-a -std=c++14'"; \
    fi

RUN cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && git checkout v0.9.1 && mkdir build && cd build && \
    cmake $PANGOLIN_CMAKE_FLAGS .. && \
    make -j8 && make install && \
    cd / && rm -rf /tmp/Pangolin

# Build vscode (can be removed later for deployment)
COPY ./container_root/shell_scripts/vscode_install.sh /root/
RUN cd /root/ && sudo chmod +x * && ./vscode_install.sh && rm -rf vscode_install.sh

# Build ORB-SLAM3 with its dependencies.
RUN apt-get update && apt-get install ros-humble-pcl-ros tmux -y
RUN apt-get install ros-humble-nav2-common x11-apps nano -y
COPY ORB_SLAM3 /home/orb/ORB_SLAM3
RUN . /opt/ros/humble/setup.sh && cd /home/orb/ORB_SLAM3 && mkdir build && ./build.sh

RUN sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp

RUN pip install flask pymap3d
RUN mkdir -p /root/colcon_ws/
RUN cd /root/colcon_ws/
RUN export ROBOT_NAMESPACE="" && export ROBOT_Y="1.0" && export ROBOT_X="1.0"
