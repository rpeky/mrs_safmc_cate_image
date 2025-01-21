#!/bin/bash

if [[ -z "${DDS_SERIAL_PORT}" ]]; then
  PORT="/dev/ttyAMA0"
else
  PORT="${DDS_SERIAL_PORT}"
fi

if [[ -z "${DDS_BAUD}" ]]; then
  BAUD="921600"
else
  BAUD="${DDS_BAUD}"
fi

echo "Starting MicroXRCEAgent on DDS_SERIAL_PORT=$PORT and DDS_BAUD=$BAUD"
MicroXRCEAgent serial -D $PORT -b $BAUD