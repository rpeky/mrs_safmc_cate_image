#!/bin/bash

#lasiest chatgpt bash script
#remember to chmod+x this script

# Exit immediately if a command exits with a non-zero status
set -e

# Source ROS 2 Humble setup script
if [ -f "/opt/ros/humble/setup.bash" ]; then
	echo "Sourcing ROS 2 Humble setup.bash..."
	source /opt/ros/humble/setup.bash
else
	echo "Error: ROS 2 Humble setup.bash not found."
	exit 1
fi

# Navigate to the parent directory
echo "Changing to the parent directory..."
cd ..

# Run colcon build
echo "Running colcon build..."
colcon build --symlink-install

# Print success message
echo "Build completed successfully!"

