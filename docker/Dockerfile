FROM ubuntu:24.04

# Set timezone (prevent interactive prompts)
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# Install essential build tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    gcc

RUN apt-get install -y libboost-all-dev

# 특정 커밋 해시나 태그 사용
ARG DYNAMIXEL_SDK_VERSION=4.0.3
ARG MOSAIC_CORE_VERSION=v1.0.0
ARG YAML_CPP_VERSION=0.9.0

# Create working directory
WORKDIR /workspace

# Clone and build DynamixelSDK (for Raspberry Pi)
RUN git clone --depth 1 --branch ${DYNAMIXEL_SDK_VERSION} \
    https://github.com/ROBOTIS-GIT/DynamixelSDK.git
WORKDIR /workspace/DynamixelSDK/c++/build/linux_sbc
RUN make install

RUN apt-get install -y libjsoncpp-dev libssl-dev libopencv-dev libwebsocketpp-dev libcpprest-dev libfmt-dev uuid-dev

RUN git clone --depth 1 --branch yaml-cpp-${YAML_CPP_VERSION}  \
    https://github.com/jbeder/yaml-cpp.git /tmp/yaml-cpp && \
    cd /tmp/yaml-cpp && \
    mkdir build && cd build && \
    cmake -DYAML_BUILD_SHARED_LIBS=ON .. && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/yaml-cpp

# Clone mosaic-core repository
WORKDIR /workspace
RUN git clone --depth 1 --branch ${MOSAIC_CORE_VERSION} \
    https://github.com/ACSL-MOSAIC/mosaic-core.git

WORKDIR /workspace/mosaic-core/third_party
RUN tar -xzvf webrtc-tarballs/webrtc-6723-arm64.tar.gz

WORKDIR /workspace/mosaic-core
RUN mkdir -p build && cd build && \
    cmake -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_TESTS=OFF .. && \
    make install

RUN ldconfig

# Copy project source
WORKDIR /workspace/project
COPY CMakeLists.txt .
COPY src ./src

# Build project
RUN mkdir -p build && cd build && \
    cmake .. && \
    make

# Set working directory
WORKDIR /app

# Run command
CMD ["/usr/local/bin/mosaic_turtlebot3_wo_ros_example"]