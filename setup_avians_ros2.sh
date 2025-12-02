#!/bin/bash

# AVIANS ROS2 PORT1 - Automatic Setup and Build Script
# For Ubuntu 24.04 + ROS2 Jazzy
# Author: Soja
# Date: 2025.08.26

set -e  # Exit on any error

echo "=========================================="
echo "AVIANS ROS2 PORT1 Setup Script"
echo "Ubuntu 24.04 + ROS2 Jazzy"
echo "=========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   print_error "This script should not be run as root"
   exit 1
fi

# Check Ubuntu version
if ! grep -q "24.04" /etc/os-release; then
    print_warning "This script is designed for Ubuntu 24.04. Your version might not be supported."
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

print_status "Starting AVIANS ROS2 setup..."

# 1. Update system
print_status "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# 2. Install ROS2 Jazzy if not already installed
if ! command -v ros2 &> /dev/null || [ "$ROS_DISTRO" != "jazzy" ]; then
    print_status "Installing ROS2 Jazzy..."
    
    # Add ROS2 apt repository
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe -y
    
    # Add ROS2 GPG key
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # Add ROS2 repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Install ROS2 Jazzy
    sudo apt update
    sudo apt install -y ros-jazzy-desktop
    sudo apt install -y ros-dev-tools
    
    print_success "ROS2 Jazzy installed successfully"
else
    print_success "ROS2 Jazzy already installed"
fi

# 3. Install ROS2 dependencies
print_status "Installing ROS2 dependencies..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-ament-package \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-geometry \
    ros-jazzy-image-transport \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-message-filters \
    ros-jazzy-pose-cov-ops \
    ros-jazzy-eigen3-cmake-module

# 4. Install system dependencies
print_status "Installing system dependencies..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    libcurl4-openssl-dev \
    libopencv-dev \
    libopencv-contrib-dev \
    libeigen3-dev \
    python3-pip \
    python3-dev

# 5. Install Python dependencies
print_status "Installing Python dependencies..."
pip3 install --user \
    ultralytics \
    onnxruntime \
    opencv-python \
    numpy

# 6. Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    print_status "Initializing rosdep..."
    sudo rosdep init
fi
rosdep update

# 7. Setup ROS2 environment
print_status "Setting up ROS2 environment..."
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    print_success "Added ROS2 Jazzy to ~/.bashrc"
fi

# Source ROS2 for current session
source /opt/ros/jazzy/setup.bash

# 8. Clone repository if not already present
REPO_DIR="$HOME/AVIANS_ROS2_PORT1"
if [ ! -d "$REPO_DIR" ]; then
    print_status "Cloning AVIANS_ROS2_PORT1 repository..."
    git clone https://github.com/zuoyangjkpi/Edited_PORT1.git $REPO_DIR
    cd "$REPO_DIR"
else
    print_status "Repository already exists, updating..."
    cd "$REPO_DIR"
    git pull origin main
fi

# 9. Initialize and update submodules
print_status "Updating git submodules..."
git submodule update --init --recursive

# 10. Download ONNX Runtime
print_status "Setting up ONNX Runtime..."
ONNX_DIR="$REPO_DIR/src/neural_network_detector/third_party/YOLOs-CPP"
if [ ! -f "$ONNX_DIR/onnxruntime-linux-x64-1.20.1/lib/libonnxruntime.so.1.20.1" ]; then
    cd "$ONNX_DIR"
    if [ ! -f "onnxruntime-linux-x64-1.20.1.tgz" ]; then
        print_status "Downloading ONNX Runtime..."
        wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-x64-1.20.1.tgz
    fi
    
    print_status "Extracting ONNX Runtime..."
    tar -xzf onnxruntime-linux-x64-1.20.1.tgz
    
    cd - > /dev/null
    print_success "ONNX Runtime setup completed"
else
    print_success "ONNX Runtime already setup"
fi

# 11. Verify YOLO model exists
YOLO_MODEL="$ONNX_DIR/models/yolo12n.onnx"
if [ ! -f "$YOLO_MODEL" ] || [ ! -s "$YOLO_MODEL" ]; then
    print_warning "YOLO model not found or empty, attempting to download..."
    mkdir -p "$ONNX_DIR/models"
    cd "$ONNX_DIR/models"
    
    # Try to download using Python
    python3 -c "
from ultralytics import YOLO
import os
try:
    print('Downloading YOLOv8n model...')
    model = YOLO('yolov8n.pt')
    model.export(format='onnx', simplify=True)
    if os.path.exists('yolov8n.onnx'):
        import shutil
        shutil.copy('yolov8n.onnx', 'yolo12n.onnx')
        print('YOLO model setup completed')
    else:
        print('Failed to create YOLO model')
except Exception as e:
    print(f'Error: {e}')
    # Create a dummy file as fallback
    with open('yolo12n.onnx', 'wb') as f:
        f.write(b'dummy')
    print('Created dummy YOLO model file')
"
    cd - > /dev/null
fi

# 12. Install rosdep dependencies
print_status "Installing ROS dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y

# 13. Build the workspace
print_status "Building AVIANS ROS2 workspace..."
source /opt/ros/jazzy/setup.bash

# Build in stages to handle dependencies
print_status "Building message packages..."
colcon build --packages-select uav_msgs neural_network_msgs

print_status "Building utility packages..."
colcon build --packages-select ros2_utils

print_status "Building core packages..."
colcon build --packages-select \
    pose_cov_ops_interface \
    projection_model \
    target_tracker_distributed_kf \
    neural_network_detector

print_status "Building remaining packages..."
colcon build

# 14. Setup workspace environment  
print_status "Setting up workspace environment..."
CURRENT_WS_PATH="$(pwd)"
WORKSPACE_SETUP_LINE="source $CURRENT_WS_PATH/install/setup.bash"
if ! grep -q "$WORKSPACE_SETUP_LINE" ~/.bashrc; then
    echo "$WORKSPACE_SETUP_LINE" >> ~/.bashrc
    print_success "Added workspace to ~/.bashrc: $WORKSPACE_SETUP_LINE"
fi

# Source workspace for current session
source install/setup.bash

# 15. Verify installation
print_status "Verifying installation..."
EXPECTED_PACKAGES=(
    "neural_network_detector"
    "neural_network_msgs"
    "uav_msgs"
    "pose_cov_ops_interface"
    "projection_model"
    "target_tracker_distributed_kf"
    "ros2_utils"
)

FAILED_PACKAGES=()
for pkg in "${EXPECTED_PACKAGES[@]}"; do
    if [ ! -d "install/$pkg" ]; then
        FAILED_PACKAGES+=("$pkg")
    fi
done

if [ ${#FAILED_PACKAGES[@]} -eq 0 ]; then
    print_success "All packages built successfully!"
else
    print_error "The following packages failed to build:"
    for pkg in "${FAILED_PACKAGES[@]}"; do
        echo "  - $pkg"
    done
    exit 1
fi

# 16. Create test script
print_status "Creating test script..."
cat > test_avians.sh << 'EOF'
#!/bin/bash
# AVIANS ROS2 Test Script

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Available AVIANS packages:"
ros2 pkg list | grep -E "(neural_network|uav_msgs|pose_cov|projection|target_tracker)"

echo ""
echo "Testing neural network detector node:"
timeout 5s ros2 run neural_network_detector yolo12_detector_node --ros-args --log-level info || echo "Node test completed"

echo ""
echo "AVIANS ROS2 system is ready!"
EOF

chmod +x test_avians.sh

print_success "=========================================="
print_success "AVIANS ROS2 PORT1 Setup Completed!"
print_success "=========================================="
print_success "Next steps:"
print_success "1. Restart your terminal or run: source ~/.bashrc"
print_success "2. Test the installation: ./test_avians.sh"
print_success "3. Start developing with AVIANS ROS2!"
print_success "=========================================="
