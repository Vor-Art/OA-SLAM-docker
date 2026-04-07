ARG CUDA_VERSION=11.8.0
ARG CUDNN_FLAVOR=cudnn8
ARG UBUNTU_VERSION=22.04

FROM nvidia/cuda:${CUDA_VERSION}-${CUDNN_FLAVOR}-devel-ubuntu${UBUNTU_VERSION}

ARG DEBIAN_FRONTEND=noninteractive
ARG OPENCV_VERSION=4.6.0
ARG NPROC=16
ARG CUDA_ARCH_BIN=7.5
ARG ROS_DISTRO=humble

ENV TZ=Europe/Moscow
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_WS=/opt/oaslam_ros2_ws

SHELL ["/bin/bash", "-c"]

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
    tree \
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
    libboost-dev \
    libboost-python-dev \
    libopengl-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    libglvnd-dev \
    libx11-dev \
    libxi-dev \
    libxmu-dev \
    gnupg2 \
    lsb-release \
    python3-dev \
    python3-pip \
    python3-numpy \
    python3-yaml \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ROS2 repository key and source
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
      > /etc/apt/sources.list.d/ros2.list

# Install ROS2 packages (separate layer to cache them independently)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-launch \
    ros-${ROS_DISTRO}-launch-ros \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-message-filters \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# CUDA symlink (cached)
RUN if [ -d /usr/local/cuda-$(echo ${CUDA_VERSION} | cut -d. -f1,2) ]; then \
      ln -sfn /usr/local/cuda-$(echo ${CUDA_VERSION} | cut -d. -f1,2) /usr/local/cuda; \
    fi

# OpenCV (heavy, rarely changes)
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

# Pangolin
RUN cd /opt && \
    git clone --depth 1 https://github.com/stevenlovegrove/Pangolin.git pangolin && \
    cd pangolin && \
    cmake -S . -B build \
      -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build --parallel ${NPROC} && \
    cmake --install build && \
    ldconfig && \
    rm -rf /opt/pangolin

# dlib
RUN cd /opt && \
    git clone --depth 1 https://github.com/davisking/dlib.git dlib && \
    cmake -S /opt/dlib -B /opt/dlib/build \
      -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /opt/dlib/build --parallel ${NPROC} && \
    cmake --install /opt/dlib/build && \
    ldconfig && \
    rm -rf /opt/dlib

# Install Sophus (header-only SE3/SO3 library required by ORB-SLAM3)
RUN git clone --depth 1 --branch 1.22.10 https://github.com/strasdat/Sophus.git /tmp/Sophus && \
    cd /tmp/Sophus && \
    mkdir build && cd build && \
    cmake .. -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/Sophus

# -------------------------------
# Build third-party dependencies (cached layer)
COPY Thirdparty/DBoW2 /opt/OA-SLAM/Thirdparty/DBoW2
RUN cd /opt/OA-SLAM/Thirdparty/DBoW2 && \
    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build --parallel ${NPROC}

COPY Thirdparty/g2o /opt/OA-SLAM/Thirdparty/g2o
RUN cd /opt/OA-SLAM/Thirdparty/g2o && \
    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build --parallel ${NPROC}

# Vocabulary (rarely changes)
COPY Vocabulary /opt/OA-SLAM/Vocabulary
RUN cd /opt/OA-SLAM/Vocabulary && tar -xf ORBvoc.txt.tar.gz

# -------------------------------
# hotfix
RUN apt-get update && apt-get install -y --no-install-recommends \
    libboost-serialization-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy CMakeLists and other build configuration files
COPY CMakeLists.txt /opt/OA-SLAM/
COPY cmake /opt/OA-SLAM/cmake

# Copy source code and headers (these change more frequently)
COPY include /opt/OA-SLAM/include
COPY src /opt/OA-SLAM/src
COPY examples /opt/OA-SLAM/examples
RUN mkdir -p /opt/OA-SLAM/Data

# Build OA-SLAM main project
# NOTE: We use "Unix Makefiles" instead of Ninja to avoid the
# "manifest 'build.ninja' still dirty after 100 tries" error that
# occurs when CMake re-generates build files (e.g. from
# configure_package_config_file) during the Ninja build step.
RUN cd /opt/OA-SLAM && \
    cmake -S . -B build \
      -G "Unix Makefiles" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCUDAToolkit_ROOT=/usr/local/cuda

RUN cd /opt/OA-SLAM && \
    cmake --build build --parallel ${NPROC} --target oaslam_legacy_internal

RUN cd /opt/OA-SLAM && \
    cmake --build build --parallel ${NPROC} --target orbslam3_g2o orbslam3_DBoW2

RUN cd /opt/OA-SLAM && \
    cmake --build build --parallel ${NPROC} --target oaslam_orbslam3_internal

RUN cd /opt/OA-SLAM && \
    cmake --build build --parallel ${NPROC} -- --output-sync=target && \
    cmake --install build && \
    ldconfig

# ROS2 wrapper (depends on OA-SLAM build above)
# hotfix
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

COPY ros2 /opt/OA-SLAM/ros2
RUN mkdir -p ${ROS_WS}/src && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros-perception/vision_opencv.git ${ROS_WS}/src/vision_opencv && \
    cp -r /opt/OA-SLAM/ros2/oaslam_ros2_wrapper ${ROS_WS}/src/oaslam_ros2_wrapper

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${ROS_WS} && \
    export CMAKE_PREFIX_PATH=/usr/local:\${CMAKE_PREFIX_PATH} && \
    colcon build --merge-install --packages-select cv_bridge oaslam_ros2_wrapper --cmake-args -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/usr/local/lib/cmake/opencv4"

ARG OASLAM_UID=1000
ARG OASLAM_GID=1000
ARG OASLAM_USER=oaslam

RUN groupadd -g "${OASLAM_GID}" "${OASLAM_USER}" && \
    useradd -m -s /bin/bash -u "${OASLAM_UID}" -g "${OASLAM_GID}" "${OASLAM_USER}" && \
    install -d -o "${OASLAM_UID}" -g "${OASLAM_GID}" "/home/${OASLAM_USER}/.ros"

WORKDIR /opt/OA-SLAM
CMD ["/bin/bash"]