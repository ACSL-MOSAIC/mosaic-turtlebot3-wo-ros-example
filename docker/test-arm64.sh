#!/bin/bash

# ARM64 이미지 테스트 스크립트 (Mac에서 에뮬레이션)

set -e

echo "Testing ARM64 image (emulated)..."
echo "Note: This will fail at USB device access, but tests the build"
echo ""

# 이미지가 존재하는지 확인
if ! docker image inspect mosaic-turtlebot3-wo-ros:arm64 > /dev/null 2>&1; then
    echo "Error: Image 'mosaic-turtlebot3-wo-ros:arm64' not found"
    echo "Please run ./build-arm64.sh first"
    exit 1
fi

# 이미지 정보 출력
echo "Image architecture:"
docker image inspect mosaic-turtlebot3-wo-ros:arm64 --format '{{.Architecture}}'

echo ""
echo "Running container (will fail at /dev/ttyACM0 access)..."
echo "Press Ctrl+C to stop"
echo ""

# 컨테이너 실행 (USB 디바이스 없이)
docker run --rm -it mosaic-turtlebot3-wo-ros:arm64