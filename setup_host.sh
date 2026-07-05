#!/bin/bash
set -e

PROJECT_DIR="/opt/main/Trajectory"
WS_DIR="$PROJECT_DIR/host_colcon_ws"

echo ">>> System dependencies"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    build-essential cmake git libeigen3-dev \
    libpng-dev libjpeg-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgtk-3-dev libgl1-mesa-dev libglew-dev \
    libvulkan1 vulkan-tools \
    python3-pip python3-dev python3-numpy \
    libboost-all-dev libssl-dev \
    python3-colcon-common-extensions python3-rosdep \
    libcanberra-gtk3-module

echo ">>> Sophus"
cd "$PROJECT_DIR/ORB_SLAM3/Thirdparty/Sophus"
rm -rf build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig

echo ">>> ORB-SLAM3"
cd "$PROJECT_DIR/ORB_SLAM3"
chmod +x build.sh
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
./build.sh
sudo bash -c "echo \"$PROJECT_DIR/ORB_SLAM3/lib\" > /etc/ld.so.conf.d/orb_slam3.conf"
sudo ldconfig

echo ">>> Colcon workspace"
mkdir -p "$WS_DIR/src"
rm -f "$WS_DIR/src/orb_slam3_ros2_wrapper" "$WS_DIR/src/slam_msgs"
ln -sf "$PROJECT_DIR/orb_slam3_ros2_wrapper" "$WS_DIR/src/"
ln -sf "$PROJECT_DIR/slam_msgs" "$WS_DIR/src/"
cp "$PROJECT_DIR/TerraSLAM_relay/slam_transform_matrix.txt" "$WS_DIR/src/orb_slam3_ros2_wrapper/" 2>/dev/null || true
cp "$PROJECT_DIR/TerraSLAM_relay/transform.json" "$WS_DIR/src/orb_slam3_ros2_wrapper/" 2>/dev/null || true

cd "$WS_DIR"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_LIBRARY_PATH=/usr/local/lib

sudo bash -c "echo \"$WS_DIR/install/lib\" > /etc/ld.so.conf.d/terraslam_host.conf"
sudo ldconfig

echo ">>> Python deps"
pip3 install --user --upgrade pip
pip3 install --user pyrealsense2 fastapi uvicorn pyyaml requests pymap3d smbus2 pyserial

echo ">>> Bashrc"
if ! grep -q "TerraSLAM Host" ~/.bashrc; then
cat >> ~/.bashrc << 'EOF'

# --- TerraSLAM Host ---
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export ROS_DOMAIN_ID=0
export QT_X11_NO_MITSHM=1
source /opt/ros/humble/setup.bash
source /opt/main/Trajectory/host_colcon_ws/install/setup.bash
# ----------------------
EOF
    echo "Added to ~/.bashrc"
fi

echo ">>> Done. Run: source ~/.bashrc"
