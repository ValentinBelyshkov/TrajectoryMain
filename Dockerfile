
FROM docker.io/arm64v8/ubuntu:22.04

# =========== NVIDIA/Jetson Environment ===========
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV DEBIAN_FRONTEND=noninteractive
ENV USERNAME=orb
ENV USER_HOME=/home/$USERNAME
ENV COLCON_WS=/home/$USERNAME/colcon_ws
ENV ROS_DISTRO=humble
ENV ROSBRIDGE_PORT=9091
# =========== User Setup ===========
ARG UID=1000
ARG GID=1000

RUN set -e && \
    if id -u ubuntu &>/dev/null; then userdel -r ubuntu 2>/dev/null || true; fi && \
    if getent group ubuntu &>/dev/null; then groupdel ubuntu 2>/dev/null || true; fi && \
    groupadd -g $GID $USERNAME && \
    useradd -u $UID -g $GID -m -s /bin/bash $USERNAME && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    usermod -aG sudo,video $USERNAME && \
    mkdir -p /etc/sudoers.d && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
    
# =========== Jetson + ROS keys (downloaded by Docker daemon on host) ===========
ADD https://gitlab.com/nvidia/container-images/l4t-base/-/raw/master/jetson-ota-public.key /etc/jetson-ota-public.key
ADD https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /tmp/ros.key

# =========== Ubuntu Mirror + ca-certificates + gnupg FIRST ===========
RUN . /etc/os-release && \
    ARCH=$(dpkg --print-architecture) && \
    if [ "$ARCH" = "arm64" ] || [ "$ARCH" = "armhf" ]; then \
        MIRROR="http://mirror.selectel.ru/ubuntu-ports"; \
    else \
        MIRROR="http://mirror.selectel.ru/ubuntu"; \
    fi && \
    printf '%s\n%s\n%s\n' \
        "deb ${MIRROR} ${VERSION_CODENAME} main restricted universe multiverse" \
        "deb ${MIRROR} ${VERSION_CODENAME}-updates main restricted universe multiverse" \
        "deb ${MIRROR} ${VERSION_CODENAME}-security main restricted universe multiverse" \
        > /etc/apt/sources.list && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates gnupg && \
    rm -rf /var/lib/apt/lists/* && \
    gpg --dearmor -o /usr/share/keyrings/jetson-ota-public.gpg /etc/jetson-ota-public.key && \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg /tmp/ros.key && \
    chmod 644 /usr/share/keyrings/*.gpg

# =========== ALL REPOS + ALL PACKAGES (single RUN) ===========
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/jetson-ota-public.gpg] https://repo.download.nvidia.com/jetson/common r36.4 main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/jetson-ota-public.gpg] https://repo.download.nvidia.com/jetson/t234 r36.4 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/jetson-ota-public.gpg] https://repo.download.nvidia.com/jetson/ffmpeg r36.4 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        bash-completion bc build-essential bzip2 can-utils cmake \
        curl emacs freeglut3-dev git gnupg2 \
        gstreamer1.0-alsa gstreamer1.0-libav gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly \
        gstreamer1.0-tools i2c-tools iproute2 iputils-ping iw kbd kmod \
        libcanberra-gtk3-module libgles2 \
        libglu1-mesa-dev libglvnd-dev libgtk-3-0 libudev1 libvulkan1 libzmq5 \
        mesa-utils mtd-utils parted pciutils python3 python3-distutils \
        python3-numpy python3-pexpect python3-pip sox sudo tmux udev \
        vulkan-tools wget wireless-tools wpasupplicant supervisor \
        cuda libcudnn9 libcudnn9-dev \
        ros-humble-desktop \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosbag2-storage-mcap \
        ros-dev-tools \
        ros-humble-pcl-ros \
        ros-humble-nav2-common \
        ros-humble-rosbridge-server \
        ros-humble-rosbridge-suite \
        ros-humble-rosidl-adapter \
        ros-humble-rosidl-default-generators \
        ros-humble-rosidl-typesupport-fastrtps-cpp \
        ros-humble-rosidl-typesupport-fastrtps-c \
        ros-humble-rmw-fastrtps-cpp \
        python3-colcon-common-extensions python3-rosdep lsb-core vim \
        libpng16-16 libjpeg-turbo8 libtiff5 python3-dev \
        libavcodec-dev libavformat-dev libswscale-dev \
        libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libgtk-3-dev \
        libgl1-mesa-dev libglew-dev libeigen3-dev apt-transport-https \
        x11-apps nano \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/* && \
    ldconfig


# =========== Build OpenCV 4.4.0 ===========
RUN cd /tmp && \
    git clone --depth 1 --branch 4.4.0 https://github.com/opencv/opencv.git && \
    cd opencv && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
          -D BUILD_EXAMPLES=OFF -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          .. && \
    make -j$(nproc) && make install && \
    cd / && rm -rf /tmp/opencv && ldconfig

# =========== Build Pangolin ===========
RUN cd /tmp && \
    git clone --depth 1 --branch v0.9.1 https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14 -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && make install && \
    cd / && rm -rf /tmp/Pangolin && ldconfig

# =========== Build Sophus ===========
COPY ORB_SLAM3/Thirdparty/Sophus /tmp/Sophus
RUN cd /tmp/Sophus && \
    rm -rf  build &&\
    mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && \
    make install && \
    cd / && rm -rf /tmp/Sophus && ldconfig
    
# =========== ORB-SLAM3 Build ===========
COPY ORB_SLAM3 /home/orb/ORB_SLAM3
RUN cd /home/orb/ORB_SLAM3 && \
    chmod +x build.sh && \
    /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && ./build.sh" && \
    ldconfig && \
    echo "/home/orb/ORB_SLAM3/lib" > /etc/ld.so.conf.d/orb_slam3.conf && \
    ldconfig
    
RUN mkdir -p $COLCON_WS/src $USER_HOME/TerraSLAM_relay $USER_HOME/Database/Mill-video /var/log/supervisor && \
    chown -R $USERNAME:$USERNAME $USER_HOME /var/log/supervisor
    
# Copy ROS2 packages (bake into image for production)
COPY orb_slam3_ros2_wrapper/ $COLCON_WS/src/orb_slam3_ros2_wrapper/
COPY slam_msgs/ $COLCON_WS/src/slam_msgs/
COPY TerraSLAM_relay/ $USER_HOME/TerraSLAM_relay/
COPY Database/ $USER_HOME/Database/
COPY TerraSLAM_relay/slam_transform_matrix.txt $COLCON_WS/src/orb_slam3_ros2_wrapper/slam_transform_matrix.txt
COPY TerraSLAM_relay/transform.json $COLCON_WS/src/orb_slam3_ros2_wrapper/transform.json
RUN chmod +x /home/orb/TerraSLAM_relay/run_relay.sh


# =========== Build ROS2 Workspace (as orb user) ===========
USER $USERNAME
WORKDIR $USER_HOME
SHELL ["/bin/bash", "-l", "-c"]

RUN echo "export PATH=/usr/local/cuda/bin:\$PATH" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64:\$LD_LIBRARY_PATH" >> ~/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source $COLCON_WS/install/setup.bash" >> ~/.bashrc && \
    rosdep update --rosdistro=$ROS_DISTRO 2>/dev/null || true
    
# Build colcon workspace
RUN /bin/bash -l -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd $COLCON_WS && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/lib"

USER root

RUN echo "/home/orb/colcon_ws/install/lib" > /etc/ld.so.conf.d/colcon_ws.conf && \
    ldconfig

USER $USERNAME
    
RUN mkdir frames
# =========== Supervisor Configuration ===========
USER root
RUN rm -rf /home/orb/.ros && \
    usermod -a -G dialout $USERNAME

RUN echo "/home/orb/ORB_SLAM3/lib" > /etc/ld.so.conf.d/orb_slam3.conf && \
    echo "/home/orb/colcon_ws/install/lib" > /etc/ld.so.conf.d/colcon_ws.conf && \
    echo "/home/orb/colcon_ws/install/orb_slam3_ros2_wrapper/lib" > /etc/ld.so.conf.d/orb_slam3_wrapper.conf && \
    ldconfig

RUN apt-get update && apt-get install -y python3-pip && \
    pip install --no-cache-dir fastapi uvicorn serial pyyaml requests pymap3d && \
    rm -rf /var/lib/apt/lists/*
        
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


# =========== Final Environment ===========
ENV PATH=/usr/local/cuda/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
ENV ROS_DOMAIN_ID=0
ENV QT_X11_NO_MITSHM=1

WORKDIR $USER_HOME
ENTRYPOINT ["/entrypoint.sh"]
