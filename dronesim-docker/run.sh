#!/bin/bash

echo "=========================================="
echo "Starting the DroneSim Docker container"
echo "=========================================="

# Allow X11 forwarding
xhost +local:docker

# Ensure shared dir exists
mkdir -p ./shared

# Start the container
docker-compose up -d

echo "Container started!"
echo ""
echo "Use the following command to enter the container:"
echo "docker exec -it dronesim-ros2-container /bin/bash"
echo ""
echo "Or simply run:"
echo "./shell.sh"
echo "=========================================="
