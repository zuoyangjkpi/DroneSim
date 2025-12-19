#!/bin/bash

echo "=========================================="
echo "Building DroneSim Docker image"
echo "=========================================="

# Allow X11 access for Docker
xhost +local:docker

# Build the Docker image
docker-compose build --no-cache

echo "=========================================="
echo "Image build complete!"
echo "Use ./run.sh to start the container"
echo "=========================================="
