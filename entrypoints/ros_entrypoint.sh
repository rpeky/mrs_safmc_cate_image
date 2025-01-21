#!/bin/bash

if [[ -z "${PATH_TO_WS}" ]]; then
  WS="safmc_ws"
else
  WS="${PATH_TO_WS}"
fi

if [[ -z "${PACKAGE_NAME}" ]]; then
  PACKAGE="safmc_2024"
else
  PACKAGE="${PACKAGE_NAME}"
fi

if [[ -z "${NODE_NAME}" ]]; then
  NODE="offboard_controller"
else
  NODE="${NODE_NAME}"
fi

echo "Starting ROS Node in $WS / $PACKAGE / $NODE"
cd /root

source /opt/ros/humble/setup.bash

source /root/$WS/install/local_setup.bash

ros2 run $PACKAGE $NODE