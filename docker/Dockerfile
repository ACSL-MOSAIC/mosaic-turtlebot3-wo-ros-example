# Base image: Ubuntu 24.04
FROM ubuntu:24.04 AS builder

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

# Create working directory
WORKDIR /workspace

# Clone and build DynamixelSDK (for Raspberry Pi)
RUN git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
WORKDIR /workspace/DynamixelSDK/c++/build/linux_sbc
RUN make install

RUN apt-get install -y libjsoncpp-dev libssl-dev libopencv-dev libwebsocketpp-dev libcpprest-dev libfmt-dev libprotobuf-dev uuid-dev

RUN git clone https://github.com/jbeder/yaml-cpp.git /tmp/yaml-cpp && \
    cd /tmp/yaml-cpp && \
    mkdir build && cd build && \
    cmake -DYAML_BUILD_SHARED_LIBS=ON .. && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/yaml-cpp

# Clone mosaic-core repository
WORKDIR /workspace
RUN git clone https://github.com/ACSL-MOSAIC/mosaic-core.git

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

# Runtime stage
FROM ubuntu:24.04

# Set timezone
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# Install runtime dependencies only
RUN apt-get update && apt-get install -y \
    libstdc++6 \
    libcpprest-dev \
    libjsoncpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy DynamixelSDK library (for Raspberry Pi)
COPY --from=builder /usr/local/lib/libdxl_sbc_cpp.so /usr/local/lib/
COPY --from=builder /usr/local/include/dynamixel_sdk /usr/local/include/dynamixel_sdk

# Copy mosaic-core libraries
COPY --from=builder /usr/local/lib/libmosaic*.so* /usr/local/lib/
COPY --from=builder /usr/local/include/mosaic /usr/local/include/mosaic

# Copy built executable
COPY --from=builder /workspace/project/build/mosaic_turtlebot3_wo_ros_example /usr/local/bin/

# Update library path
RUN ldconfig

# Set working directory
WORKDIR /app

# Run command
CMD ["/usr/local/bin/mosaic_turtlebot3_wo_ros_example"]