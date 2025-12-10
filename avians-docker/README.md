# AVIANS ROS2 Docker Environment

This Docker setup ships with the complete AVIANS ROS2 PORT1 workspace, including:

- Ubuntu 24.04 base image
- Full ROS2 Jazzy desktop install
- `airship_ros2` conda environment
- All project dependencies and packages
- Pre-built ROS2 workspace

## Quick Start

### 1. Build the image

```bash
cd ~/avians-docker
./build.sh
```

### 2. Start the container

```bash
./run.sh
```

### 3. Enter the container

```bash
./shell.sh
```

### 4. Stop the container

```bash
./stop.sh
```

## Directory Layout

```
~/avians-docker/
├── Dockerfile              # Docker image definition
├── docker-compose.yml      # Docker Compose config
├── airship_ros2_env.yml    # Conda environment spec
├── build.sh                # Build script
├── run.sh                  # Launch script
├── shell.sh                # Attach script
├── stop.sh                 # Stop script
├── shared/                 # Shared host/container folder
└── README.md               # This document
```

## Container Environment

- **User**: `aviansuser` (non-root, passwordless sudo)
- **Workspace**: `/home/aviansuser/AVIANS_ROS2_PORT1`
- **Conda env**: `airship_ros2` (auto-activated)
- **ROS2**: Jazzy
- **Network mode**: host (shares the host network)

## Environment Setup

The container automatically runs:

```bash
source /opt/ros/jazzy/setup.bash
source /home/aviansuser/AVIANS_ROS2_PORT1/install/setup.bash
conda activate airship_ros2
```

## Usage Notes

### Once inside the container

```bash
# List project ROS2 packages
ros2 pkg list | grep -E "(neural_network|uav_msgs|pose_cov|projection|target_tracker)"

# Run tests
cd /home/aviansuser/AVIANS_ROS2_PORT1
./test_avians.sh

# Launch a detector node
ros2 run neural_network_detector yolo12_detector_node
```

### Development workflow

Use the `shared/` directory to exchange files between the host and container during development.

### Data persistence

The workspace and conda environment live in Docker volumes, so data survives container removal.

## Troubleshooting

### GUI apps fail to display

Allow X11 forwarding:

```bash
xhost +local:docker
```

### Permission issues

Commands run as `aviansuser`, who has passwordless sudo inside the container.

### Network conflicts

The container uses host networking. Adjust `docker-compose.yml` if you need a different networking mode.

## Customization

Feel free to tweak:

- `Dockerfile`: base image, dependencies, entrypoint
- `docker-compose.yml`: ports, volumes, runtime options
- `airship_ros2_env.yml`: Python dependencies

## Moving to a New Machine

1. Copy the entire `avians-docker` folder to the new system.
2. Install Docker and Docker Compose.
3. Run `./build.sh` to build the image.
4. Run `./run.sh` to start the container.
5. Run `./shell.sh` to begin working.

You're ready to use AVIANS ROS2 on the new computer!
