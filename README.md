# ScoutSLAM

ScoutSLAM is an object-aided visual SLAM codebase built around an ORB-SLAM3
backend, reusable C++ session modules, and a ROS 2 Humble wrapper for RGB-D /
VIO-style online and offline runs.

This repository currently uses Docker as the supported development and runtime
environment.

## Requirements

- Docker with Compose v2.
- NVIDIA driver and NVIDIA Container Toolkit for GPU/CUDA execution.
- X11 access if `use_viewer: true` is enabled in the ROS 2 parameters.

The image defaults are configured in `docker/.env`:

```env
CUDA_VERSION=11.8.0
CUDNN_FLAVOR=cudnn8
UBUNTU_VERSION=22.04
OPENCV_VERSION=4.6.0
ROS_DISTRO=humble
CUDA_ARCH_BIN=7.5
```

## Quick Start

From the repository root:

```bash
./docker/run.sh build
./docker/run.sh start
./docker/run.sh rebuild
```

What these commands do:

- `build` builds the Docker image.
- `start` starts the long-running `scoutslam` container with the repository.
- `rebuild` builds and installs the C++ ScoutSLAM libraries, then builds the ROS 2 workspace inside the container.

For viewer windows, allow the container to use the host X server before running:

```bash
xhost +local:root
```

## ROS 2 Online Run

The online node subscribes to RGB, depth, and IMU topics and publishes pose,
map point, semantic map, local semantic map, and marker outputs.

Default config:

```bash
./docker/run.sh run-online
```

Run with a custom parameter file inside the container:

```bash
./docker/run.sh run-online /opt/OA-SLAM/ros2/oaslam_ros2_wrapper/config/online_vio.yaml
```

## ROS 2 Offline Run

The offline node reads a ROS 2 bag directory directly and processes messages sequentially without requiring `ros2 bag play`.

Default config:

```bash
./docker/run.sh run-offline
```

To run another bag, edit `bag_path` in a parameter file under `ros2/` or `Data/`
and pass that container path:

```bash
./docker/run.sh run-offline /opt/OA-SLAM/ros2/oaslam_ros2_wrapper/config/offline_vio.yaml
```

The bag must contain the configured RGB and depth topics. The IMU topic is read
when available; if it is missing, the offline node logs a warning and continues.

## Data, Models, and Outputs

The current checked-in data layout includes:

- `Data/yolov5m_640x480.onnx` - default ONNX object detector path used by the
  sample parameter files.
- `Data/run/` - trajectory output directory.
- `Data/rviz_cfg.rviz` - RViz configuration.

## Important parameters

Important parameters:

- `rgb_topic`, `depth_topic`, `imu_topic` - input topics.
- `pose_topic`, `map_points_topic`, `semantic_map_*` - legacy output topics.
- `local_semantic_map_*` - SharedSemanticMap-compatible per-agent semantic
  topics, for example `/agent_1/semantic_map_delta`.
- `vocabulary_file` - usually `/opt/ScoutSLAM/Vocabulary/ORBvoc.txt`.
- `camera_settings_file` - camera/ORB-SLAM settings YAML.
- `observation_mode` - `onnx`, `file`, or `none`.
- `detection_model_path` - required when `observation_mode: "onnx"`.
- `detection_file_path` - required when `observation_mode: "file"`.
- `ignored_categories_file` - optional category ignore list.
- `relocalization_mode` - `points`, `objects`, or `points_and_objects`.
- `use_viewer` - enables Pangolin viewer windows.
- `use_imu` - enables IMU use in the SLAM backend.
- `output_folder` - base folder for timestamped TUM trajectory outputs.

## License

ScoutSLAM is released under GPLv3. The codebase derives from ScoutSLAM, ORB-SLAM2, and ORB-SLAM3 components; see `LICENSE` and bundled third-party directories for
details.
