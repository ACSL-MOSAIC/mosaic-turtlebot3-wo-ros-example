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

RUN rm -rf /var/lib/apt/lists/*

# Create working directory
WORKDIR /workspace

# Clone and build DynamixelSDK (for Raspberry Pi)
RUN git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
WORKDIR /workspace/DynamixelSDK/c++/build/linux_sbc
RUN make install

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
    && rm -rf /var/lib/apt/lists/*

# Copy DynamixelSDK library (for Raspberry Pi)
COPY --from=builder /usr/local/lib/libdxl_sbc_cpp.so /usr/local/lib/
COPY --from=builder /usr/local/include/dynamixel_sdk /usr/local/include/dynamixel_sdk

# Copy built executable
COPY --from=builder /workspace/project/build/mosaic_turtlebot3_wo_ros_example /usr/local/bin/

# Update library path
RUN ldconfig

# Set working directory
WORKDIR /app

# Run command
CMD ["/usr/local/bin/mosaic_turtlebot3_wo_ros_example"]