#!/bin/bash

# Docker 이미지 빌드 스크립트

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"

echo "Building Docker image..."
cd "$SCRIPT_DIR"
docker-compose build

echo ""
echo "Build completed successfully!"
echo "To run the container, use: ./run.sh"