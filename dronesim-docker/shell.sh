#!/bin/bash

echo "Entering the DroneSim container..."

# Attach to the running container
docker exec -it dronesim-ros2-container /bin/bash
