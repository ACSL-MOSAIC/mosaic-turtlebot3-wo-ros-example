#!/bin/bash

# Docker 컨테이너 실행 스크립트

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Starting TurtleBot3 controller..."
cd "$SCRIPT_DIR"

# USB 디바이스 확인
if [ ! -e /dev/ttyACM0 ]; then
    echo "Warning: /dev/ttyACM0 not found!"
    echo "Please check if the device is connected."
    exit 1
fi

# 컨테이너 실행
docker-compose up

# Ctrl+C로 종료 시 컨테이너 정리
trap "docker-compose down" EXIT