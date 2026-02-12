#!/bin/bash

# ARM64 image test script (emulated on Mac)

set -e

echo "Testing ARM64 image (emulated)..."
echo "Note: This will fail at USB device access, but tests the build"
echo ""

# Check if image exists
if ! docker image inspect mosaic-turtlebot3-wo-ros:arm64 > /dev/null 2>&1; then
    echo "Error: Image 'mosaic-turtlebot3-wo-ros:arm64' not found"
    echo "Please run ./build-arm64.sh first"
    exit 1
fi

# Print image information
echo "Image architecture:"
docker image inspect mosaic-turtlebot3-wo-ros:arm64 --format '{{.Architecture}}'

echo ""
echo "Running container (will fail at /dev/ttyACM0 access)..."
echo "Press Ctrl+C to stop"
echo ""

# Run container (without USB device)
docker run --rm -it mosaic-turtlebot3-wo-ros:arm64