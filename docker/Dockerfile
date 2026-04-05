ARG CUDA_VERSION=11.8.0
ARG CUDNN_FLAVOR=cudnn8
ARG UBUNTU_VERSION=22.04

FROM nvidia/cuda:${CUDA_VERSION}-${CUDNN_FLAVOR}-devel-ubuntu${UBUNTU_VERSION}

ARG DEBIAN_FRONTEND=noninteractive
ARG OPENCV_VERSION=4.6.0
ARG NPROC=16
ARG CUDA_ARCH_BIN=7.5

ENV TZ=Europe/Moscow
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

SHELL ["/bin/bash", "-c"]

# todo delete   protobuf-compiler libprotobuf-dev 
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    ca-certificates \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    git \
    wget \
    curl \
    unzip \
    yasm \
    vim \
    software-properties-common \
    libeigen3-dev \
    libopenblas-dev \
    liblapack-dev \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    zlib1g-dev \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libpostproc-dev \
    libswscale-dev \
    libgtk2.0-dev \
    libglew-dev \
    libepoxy-dev \
    libxine2-dev \
    libpq-dev \
    protobuf-compiler \
    libprotobuf-dev \
    python3-dev \
    python3-pip \
    python3-yaml \
    tree \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    libopengl-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    libglvnd-dev \
    libx11-dev \
    libxi-dev \
    libxmu-dev \
    && rm -rf /var/lib/apt/lists/*

RUN if [ -d /usr/local/cuda-$(echo ${CUDA_VERSION} | cut -d. -f1,2) ]; then \
      ln -sfn /usr/local/cuda-$(echo ${CUDA_VERSION} | cut -d. -f1,2) /usr/local/cuda; \
    fi

RUN cd /opt && \
    wget -q https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip -O opencv.zip && \
    unzip -q opencv.zip && rm opencv.zip && \
    wget -q https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip -O opencv_contrib.zip && \
    unzip -q opencv_contrib.zip && rm opencv_contrib.zip && \
    mkdir -p /opt/opencv-${OPENCV_VERSION}/build && \
    cd /opt/opencv-${OPENCV_VERSION}/build && \
    cmake .. \
      -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DOPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib-${OPENCV_VERSION}/modules \
      -DWITH_CUDA=ON \
      -DWITH_CUDNN=ON \
      -DOPENCV_DNN_CUDA=ON \
      -DCUDA_ARCH_BIN=${CUDA_ARCH_BIN} \
      -DBUILD_TESTS=OFF \
      -DBUILD_PERF_TESTS=OFF \
      -DBUILD_EXAMPLES=OFF && \
    ninja -j${NPROC} && \
    ninja install && \
    ldconfig && \
    rm -rf /opt/opencv-${OPENCV_VERSION} /opt/opencv_contrib-${OPENCV_VERSION}

RUN cd /opt && \
    git clone --depth 1 https://github.com/stevenlovegrove/Pangolin.git pangolin && \
    cd pangolin && \
    cmake -S . -B build \
      -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build --parallel ${NPROC} && \
    cmake --install build && \
    ldconfig

RUN cd /opt && \
    git clone --depth 1 https://github.com/davisking/dlib.git dlib && \
    cmake -S /opt/dlib -B /opt/dlib/build \
      -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /opt/dlib/build --parallel ${NPROC} && \
    cmake --install /opt/dlib/build && \
    ldconfig

# Build third-party dependencies (cached layer)
COPY Thirdparty/DBoW2 /opt/OA-SLAM/Thirdparty/DBoW2
RUN cd /opt/OA-SLAM/Thirdparty/DBoW2 && \
    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build --parallel ${NPROC}

COPY Thirdparty/g2o /opt/OA-SLAM/Thirdparty/g2o
RUN cd /opt/OA-SLAM/Thirdparty/g2o && \
    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build --parallel ${NPROC}

# Copy vocabulary and extract (rarely changes)
COPY Vocabulary /opt/OA-SLAM/Vocabulary
RUN cd /opt/OA-SLAM/Vocabulary && tar -xf ORBvoc.txt.tar.gz

# Copy CMakeLists and other build configuration files
COPY CMakeLists.txt /opt/OA-SLAM/

# Copy source code and headers (these change more frequently)
COPY include /opt/OA-SLAM/include
COPY src /opt/OA-SLAM/src
COPY Examples /opt/OA-SLAM/Examples
COPY Cameras /opt/OA-SLAM/Cameras
RUN mkdir -p /opt/OA-SLAM/Data

# Build OA-SLAM main project
RUN cd /opt/OA-SLAM && \
    cmake -S . -B build \
      -G Ninja \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_BUILD_TYPE=Release \
      -DCUDAToolkit_ROOT=/usr/local/cuda && \
    cmake --build build --parallel ${NPROC} && \
    ldconfig

WORKDIR /opt/OA-SLAM
CMD ["/bin/bash"]
