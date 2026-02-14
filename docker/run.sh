#!/bin/bash

# Docker container run script

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Starting TurtleBot3 controller..."
cd "$SCRIPT_DIR"

# Check USB device
if [ ! -e /dev/ttyACM0 ]; then
    echo "Warning: /dev/ttyACM0 not found!"
    echo "Please check if the device is connected."
    exit 1
fi

# Run container
docker compose up

# Clean up container on Ctrl+C
trap "docker compose down" EXIT