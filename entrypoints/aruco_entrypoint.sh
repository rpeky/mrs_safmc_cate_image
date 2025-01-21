#!/bin/bash

if [[ -z "${PATH_TO_WS}" ]]; then
  WS="safmc_ws"
else
  WS="${PATH_TO_WS}"
fi

echo "Starting ROS Node in $WS / $PACKAGE / $NODE"
cd /root

source /opt/ros/humble/setup.bash

export LD_LIBRARY_PATH="/usr/local/lib/aarch64-linux-gnu/:$LD_LIBRARY_PATH"

source /root/$WS/install/local_setup.bash

ros2 launch safmc_2024 cameras.py
