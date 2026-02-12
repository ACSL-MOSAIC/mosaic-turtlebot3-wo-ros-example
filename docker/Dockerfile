# Base image: Ubuntu 24.04
FROM ubuntu:24.04 AS builder

# 타임존 설정 (interactive 프롬프트 방지)
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# 필수 빌드 도구 설치
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# 작업 디렉토리 생성
WORKDIR /workspace

# DynamixelSDK git clone 및 빌드 (라즈베리파이용)
RUN git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
WORKDIR /workspace/DynamixelSDK/c++/build/linux_sbc
RUN make install

RUN ldconfig

# 프로젝트 소스 복사
WORKDIR /workspace/project
COPY CMakeLists.txt .
COPY src ./src

# 프로젝트 빌드
RUN mkdir -p build && cd build && \
    cmake .. && \
    make

# Runtime stage
FROM ubuntu:24.04

# 타임존 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# 런타임 필수 패키지만 설치
RUN apt-get update && apt-get install -y \
    libstdc++6 \
    && rm -rf /var/lib/apt/lists/*

# DynamixelSDK 라이브러리 복사 (라즈베리파이용)
COPY --from=builder /usr/local/lib/libdxl_sbc_cpp.so /usr/local/lib/
COPY --from=builder /usr/local/include/dynamixel_sdk /usr/local/include/dynamixel_sdk

# 빌드된 실행 파일 복사
COPY --from=builder /workspace/project/build/mosaic_turtlebot3_wo_ros_example /usr/local/bin/

# 라이브러리 경로 업데이트
RUN ldconfig

# 작업 디렉토리 설정
WORKDIR /app

# 실행 명령
CMD ["/usr/local/bin/mosaic_turtlebot3_wo_ros_example"]