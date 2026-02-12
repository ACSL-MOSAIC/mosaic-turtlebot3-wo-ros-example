#!/bin/bash

# Docker image build script for ARM64 (Raspberry Pi)

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"

echo "Setting up Docker Buildx..."

# Create Buildx builder (use existing if available)
if ! docker buildx inspect arm-builder > /dev/null 2>&1; then
    echo "Creating new buildx builder 'arm-builder'..."
    docker buildx create --name arm-builder --use
else
    echo "Using existing buildx builder 'arm-builder'..."
    docker buildx use arm-builder
fi

# Bootstrap builder
docker buildx inspect --bootstrap

echo ""
echo "Building ARM64 image for Raspberry Pi..."
cd "$SCRIPT_DIR"

# Build for ARM64 platform and load to local
docker buildx build \
    --platform linux/arm64 \
    --tag mosaic-turtlebot3-wo-ros:arm64 \
    --load \
    -f Dockerfile \
    ..

echo ""
echo "Build completed successfully!"
echo "Image: mosaic-turtlebot3-wo-ros:arm64"
echo ""
echo "To test (without USB device):"
echo "  docker run --rm mosaic-turtlebot3-wo-ros:arm64"
echo ""
echo "To save image for Raspberry Pi:"
echo "  docker save mosaic-turtlebot3-wo-ros:arm64 | gzip > turtlebot3-arm64.tar.gz"
echo "  # Then copy to Raspberry Pi and load with:"
echo "  # docker load < turtlebot3-arm64.tar.gz"