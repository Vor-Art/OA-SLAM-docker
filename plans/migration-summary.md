# ORB-SLAM3 Migration & ORB-SLAM2 Removal Summary

> **Status**: Implementation complete — build verification and runtime testing pending  
> **Date**: 2026-04-06  
> **Reference plans**: [`orbslam3-migration-plan.md`](orbslam3-migration-plan.md), [`migration-subtasks.md`](migration-subtasks.md)

---

## 1. Executive Summary

**Goal**: Replace ORB-SLAM2 with ORB-SLAM3 inside the OA-SLAM pipeline to enable Visual-Inertial Odometry (VIO) using an Intel RealSense D435i camera (RGB-D + BMI055 IMU).

**Architecture**: The project follows a hexagonal / ports-and-adapters pattern. The core interface [`ISlamBackend`](../include/oaslam/ports/slam_backend.h) remains unchanged — ORB-SLAM3 is the sole backend adapter.

**ORB-SLAM2 removal**: The ORB-SLAM2 adapter has been completely removed. The `SlamBackendKind` enum and runtime backend selection have been eliminated. The build system no longer has any ORB-SLAM2 targets, conditional `BUILD_ORBSLAM3` option, or references to the legacy `Thirdparty/DBoW2` and `Thirdparty/g2o` shared libraries.

**Key outcome**: A single `OrbSlam3BackendAdapter` supports both plain RGBD and IMU_RGBD modes. All OA-SLAM semantic features — object tracking, ellipsoid mapping, object-based relocalization — have been ported to ORB-SLAM3. A new ROS2 VIO node handles synchronized RGB+Depth+IMU input from the D435i.

---

## 2. Implementation Phases Completed

### Phase 1: Core Type Extensions

Extended three core domain headers to carry IMU data through the pipeline without changing the `ISlamBackend` interface.

| File | Changes |
|------|---------|
| [`include/oaslam/core/frame_packet.h`](../include/oaslam/core/frame_packet.h) | Added `ImuMeasurement` struct with `timestamp`, `acc_x/y/z`, `gyro_x/y/z`; added `imu_measurements` vector and `has_imu` bool to `FramePacket` |
| [`include/oaslam/core/session_types.h`](../include/oaslam/core/session_types.h) | Added `use_imu` field to `SlamBackendConfig` |
| [`include/oaslam/core/tracking_types.h`](../include/oaslam/core/tracking_types.h) | Added `imu_initialized` bool and `velocity_x/y/z` doubles to `TrackingResult` |

### Phase 2: ORB-SLAM3 Adapter

Created the adapter layer that bridges the OA-SLAM port interface to ORB-SLAM3 internals.

| File | Purpose |
|------|---------|
| [`src/adapters/orbslam3/orbslam3_pose_utils.h`](../src/adapters/orbslam3/orbslam3_pose_utils.h) | `SophusTcwToTransform4d()` — inverts Tcw→Twc, float→double; `IsValidPose()` — detects tracking failure; `EigenToPoint3d()` — MapPoint conversion; `ToImuPoints()` — ImuMeasurement→IMU::Point |
| [`src/adapters/orbslam3/orbslam3_pose_utils.cc`](../src/adapters/orbslam3/orbslam3_pose_utils.cc) | Implementation of `ToImuPoints()` requiring full ORB_SLAM3::IMU::Point definition |
| [`src/adapters/orbslam3/orbslam3_backend_adapter.h`](../src/adapters/orbslam3/orbslam3_backend_adapter.h) | `OrbSlam3BackendAdapter` class — implements `ISlamBackend` with constructor taking vocab_path, settings_path, use_imu, use_viewer |
| [`src/adapters/orbslam3/orbslam3_backend_adapter.cc`](../src/adapters/orbslam3/orbslam3_backend_adapter.cc) | Full `processFrame()` implementation: calls `TrackRGBD()` with IMU data, extracts Sophus pose, converts to Transform4d, extracts tracked MapPoints as Eigen::Vector3f→cv::Point3d |
| [`src/app/module_factories.cc`](../src/app/module_factories.cc) | Factory wiring: directly creates `OrbSlam3BackendAdapter`; quit-check callback wired to visualizer |

### Phase 3: D435i Configuration

| File | Purpose |
|------|---------|
| [`ros2/oaslam_ros2_wrapper/config/d435i_imu_rgbd.yaml`](../ros2/oaslam_ros2_wrapper/config/d435i_imu_rgbd.yaml) | ORB-SLAM3 format YAML with `Camera1.*` pinhole intrinsics for D435i, `IMU.*` noise parameters for BMI055, `IMU.T_b_c1` extrinsic matrix, ORB extractor settings, Viewer parameters |

### Phase 4: OA-SLAM Patch Porting (20+ files)

All OA-SLAM semantic features were ported from the ORB-SLAM2 internal tree to the ORB-SLAM3 internal tree. Key adaptations:

- **Namespace**: `ORB_SLAM2` → `ORB_SLAM3` throughout
- **Pose types**: `cv::Mat` 4×4 → `Sophus::SE3f` with helper `SE3fToMatrix34d()`
- **MapPoint positions**: `cv::Mat` 3×1 → `Eigen::Vector3f` (direct `.cast<double>()`)
- **Map access**: Single `Map*` → `Atlas::GetCurrentMap()` for MapObject operations

#### T4.1: Geometry Utilities — 7 header+source pairs

| Header | Source | Notes |
|--------|--------|-------|
| [`internal/include/Ellipsoid.h`](../src/adapters/orbslam3/internal/include/Ellipsoid.h) | [`internal/src/Ellipsoid.cc`](../src/adapters/orbslam3/internal/src/Ellipsoid.cc) | Pure Eigen math, namespace rename |
| [`internal/include/Ellipse.h`](../src/adapters/orbslam3/internal/include/Ellipse.h) | [`internal/src/Ellipse.cc`](../src/adapters/orbslam3/internal/src/Ellipse.cc) | Pure Eigen math, namespace rename |
| [`internal/include/Distance.h`](../src/adapters/orbslam3/internal/include/Distance.h) | [`internal/src/Distance.cc`](../src/adapters/orbslam3/internal/src/Distance.cc) | Pure Eigen math, namespace rename |
| [`internal/include/Camera.h`](../src/adapters/orbslam3/internal/include/Camera.h) | [`internal/src/Camera.cc`](../src/adapters/orbslam3/internal/src/Camera.cc) | Namespace rename |
| [`internal/include/ColorManager.h`](../src/adapters/orbslam3/internal/include/ColorManager.h) | [`internal/src/ColorManager.cc`](../src/adapters/orbslam3/internal/src/ColorManager.cc) | Namespace rename |
| [`internal/include/Reconstruction.h`](../src/adapters/orbslam3/internal/include/Reconstruction.h) | [`internal/src/Reconstruction.cc`](../src/adapters/orbslam3/internal/src/Reconstruction.cc) | Namespace rename |
| [`internal/include/Utils.h`](../src/adapters/orbslam3/internal/include/Utils.h) | [`internal/src/Utils.cc`](../src/adapters/orbslam3/internal/src/Utils.cc) | Namespace rename |

#### T4.2: ObjectTrack and MapObject — Sophus/Eigen adaptation

| Header | Source | Key Changes |
|--------|--------|-------------|
| [`internal/include/ObjectTrack.h`](../src/adapters/orbslam3/internal/include/ObjectTrack.h) | [`internal/src/ObjectTrack.cc`](../src/adapters/orbslam3/internal/src/ObjectTrack.cc) | `SE3fToMatrix34d()` helper for pose extraction; `MapPoint::GetWorldPos()` returns `Eigen::Vector3f` instead of `cv::Mat`; Atlas-aware Map access |
| [`internal/include/MapObject.h`](../src/adapters/orbslam3/internal/include/MapObject.h) | [`internal/src/MapObject.cc`](../src/adapters/orbslam3/internal/src/MapObject.cc) | Namespace rename, KeyFrame forward declaration updated |

#### T4.3: ImageDetections — namespace rename

| Header | Source |
|--------|--------|
| [`internal/include/ImageDetections.h`](../src/adapters/orbslam3/internal/include/ImageDetections.h) | [`internal/src/ImageDetections.cc`](../src/adapters/orbslam3/internal/src/ImageDetections.cc) |

#### T4.4: Map.h/cc — MapObject storage

| File | Changes |
|------|---------|
| [`internal/include/Map.h`](../src/adapters/orbslam3/internal/include/Map.h) | Added `std::set<MapObject*> map_objects_`; `AddMapObject()`, `EraseMapObject()`, `GetAllMapObjects()`, `GetNumberMapObjects()` |
| [`internal/src/Map.cc`](../src/adapters/orbslam3/internal/src/Map.cc) | Implementation of MapObject storage methods |

#### T4.5: LocalObjectMapping thread

| Header | Source |
|--------|--------|
| [`internal/include/LocalObjectMapping.h`](../src/adapters/orbslam3/internal/include/LocalObjectMapping.h) | [`internal/src/LocalObjectMapping.cc`](../src/adapters/orbslam3/internal/src/LocalObjectMapping.cc) |

#### T4.6: Localization (P3P RANSAC, OptimizePoseFromObjects) + p3p

| Header | Source |
|--------|--------|
| [`internal/include/Localization.h`](../src/adapters/orbslam3/internal/include/Localization.h) | [`internal/src/Localization.cc`](../src/adapters/orbslam3/internal/src/Localization.cc) |
| [`internal/include/p3p.h`](../src/adapters/orbslam3/internal/include/p3p.h) | [`internal/src/p3p.cc`](../src/adapters/orbslam3/internal/src/p3p.cc) |

#### T4.7: OptimizerObject (g2o edge types) and RingBuffer

| Header | Source |
|--------|--------|
| [`internal/include/OptimizerObject.h`](../src/adapters/orbslam3/internal/include/OptimizerObject.h) | [`internal/src/OptimizerObject.cc`](../src/adapters/orbslam3/internal/src/OptimizerObject.cc) |
| [`internal/include/RingBuffer.h`](../src/adapters/orbslam3/internal/include/RingBuffer.h) | Header-only template container |

#### T4.8: System.h/cc — relocalization modes, LocalObjectMapping, detection passing

| File | Key Patches |
|------|-------------|
| [`internal/include/System.h`](../src/adapters/orbslam3/internal/include/System.h) | `enumRelocalizationMode`; `TrackRGBD()` extended with `Detection::Ptr` vector; `LocalObjectMapping*` thread member; `relocalization_duration` / `relocalization_status` fields |
| [`internal/src/System.cc`](../src/adapters/orbslam3/internal/src/System.cc) | Detection passing to Tracker; LocalObjectMapping thread lifecycle; relocalization mode dispatch |

#### T4.9: Tracking.h/cc — object tracking block, RelocalizationFromObjects, ORBmatcher

| File | Key Patches |
|------|-------------|
| [`internal/include/Tracking.h`](../src/adapters/orbslam3/internal/include/Tracking.h) | Added `objectTracks_`, `current_frame_idx_`, detection members; `RelocalizationFromObjects()` declaration |
| [`internal/src/Tracking.cc`](../src/adapters/orbslam3/internal/src/Tracking.cc) | Object tracking block (~260 lines) in `GrabImageRGBD()` — Hungarian algorithm matching, track lifecycle, ellipsoid reconstruction; `RelocalizationFromObjects()` (~90 lines) — P3P RANSAC with Sophus pose construction; relocalization mode dispatch in `Track()`; `SearchByProjectionAllPoints` in ORBmatcher |

### Phase 5: ROS2 VIO Node

| File | Purpose |
|------|---------|
| [`ros2/oaslam_ros2_wrapper/src/imu_buffer.h`](../ros2/oaslam_ros2_wrapper/src/imu_buffer.h) | Thread-safe `ImuBuffer` class with `push()` at 200Hz and `drainUntil(timestamp)` for frame sync; bounded to 2000 samples |
| [`ros2/oaslam_ros2_wrapper/src/oaslam_vio_node.cpp`](../ros2/oaslam_ros2_wrapper/src/oaslam_vio_node.cpp) | `OaSlamVioNode` — `message_filters::ApproximateTimeSynchronizer` for RGB+Depth sync; independent IMU subscription; builds `FramePacket` with drained IMU data; publishes `geometry_msgs::PoseStamped` |
| [`ros2/oaslam_ros2_wrapper/launch/oaslam_vio.launch.py`](../ros2/oaslam_ros2_wrapper/launch/oaslam_vio.launch.py) | Launch file with `params_file`, `use_sim_time`, topic remapping arguments |
| [`ros2/oaslam_ros2_wrapper/config/wrapper_vio.yaml`](../ros2/oaslam_ros2_wrapper/config/wrapper_vio.yaml) | Parameter file: vocabulary, camera settings, topics, observation mode, relocalization mode |
| [`ros2/oaslam_ros2_wrapper/CMakeLists.txt`](../ros2/oaslam_ros2_wrapper/CMakeLists.txt) | Added `oaslam_vio_node` executable target with `message_filters` dependency |
| [`ros2/oaslam_ros2_wrapper/package.xml`](../ros2/oaslam_ros2_wrapper/package.xml) | Added `message_filters` and `std_msgs` dependencies |
| [`docker/Dockerfile`](../docker/Dockerfile) | Added Sophus v1.22.10 installation; added `ros-humble-message-filters` package |

### Phase 6: Build Integration & ORB-SLAM2 Removal

| File | Changes |
|------|---------|
| [`CMakeLists.txt`](../CMakeLists.txt) | ORB-SLAM3 is the sole backend — no `BUILD_ORBSLAM3` option, no conditional blocks. Targets: `orbslam3_g2o` (static, 30 sources), `orbslam3_DBoW2` (static, 6 sources), `oaslam_orbslam3_internal` (static, 34 sources + 2 camera models), `oaslam_adapter_orbslam3` (static, 2 sources). `oaslam_adapter_observations` links against `oaslam_orbslam3_internal`. |
| [`docker/Dockerfile`](../docker/Dockerfile) | Removed ORB-SLAM2 `Thirdparty/DBoW2` and `Thirdparty/g2o` build steps (no longer needed — ORB-SLAM3 builds its own as static libs) |

---

## 3. ORB-SLAM2 Removal Details

The following ORB-SLAM2 artifacts were removed or cleaned up:

### 3.1 Deleted

| Item | Description |
|------|-------------|
| `src/adapters/orbslam2/` | Entire ORB-SLAM2 adapter directory (backend adapter + internal ORB-SLAM2 sources) |

### 3.2 Removed from Build System

| Item | Was | Now |
|------|-----|-----|
| `BUILD_ORBSLAM3` CMake option | `option(BUILD_ORBSLAM3 ... ON)` | Removed — ORB-SLAM3 always built |
| `oaslam_legacy_internal` target | Static lib from ORB-SLAM2 internal sources | Removed |
| `oaslam_adapter_orbslam2` target | Static lib from ORB-SLAM2 adapter | Removed |
| `LEGACY_INTERNAL_DIR/SOURCES` variables | Pointed to `src/adapters/orbslam2/internal/` | Removed |
| `Thirdparty/DBoW2` link | `${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so` | Removed (ORB-SLAM3 uses `orbslam3_DBoW2` static lib) |
| `Thirdparty/g2o` link | `${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so` | Removed (ORB-SLAM3 uses `orbslam3_g2o` static lib) |
| `if(BUILD_ORBSLAM3)` conditionals | Wrapped ORB-SLAM3 targets and linking | Removed — all unconditional |
| `add_definitions(-DBUILD_ORBSLAM3)` | Compile definition for conditional compilation | Removed |
| Dockerfile `Thirdparty/DBoW2` build | COPY + cmake build of ORB-SLAM2's DBoW2 | Removed |
| Dockerfile `Thirdparty/g2o` build | COPY + cmake build of ORB-SLAM2's g2o | Removed |

### 3.3 Code Cleanup

| File | Change |
|------|--------|
| [`include/oaslam/core/session_types.h`](../include/oaslam/core/session_types.h) | Removed `SlamBackendKind` enum and `kind` field from `SlamBackendConfig` |
| [`src/app/module_factories.cc`](../src/app/module_factories.cc) | Removed ORB-SLAM2 include, removed switch/case on `SlamBackendKind`, directly creates `OrbSlam3BackendAdapter` |
| [`src/adapters/observations/file_observation_source.h`](../src/adapters/observations/file_observation_source.h) | `ORB_SLAM2` → `ORB_SLAM3` namespace |
| [`src/adapters/observations/file_observation_source.cc`](../src/adapters/observations/file_observation_source.cc) | Include path redirected to orbslam3, `ORB_SLAM2` → `ORB_SLAM3` namespace |
| [`src/adapters/observations/onnx_observation_source.h`](../src/adapters/observations/onnx_observation_source.h) | `ORB_SLAM2` → `ORB_SLAM3` namespace |
| [`src/adapters/observations/onnx_observation_source.cc`](../src/adapters/observations/onnx_observation_source.cc) | Include path redirected to orbslam3, `ORB_SLAM2` → `ORB_SLAM3` namespace |
| [`src/common/opencv_utils.h`](../src/common/opencv_utils.h) | `ORB_SLAM2::BBox2` → `ORB_SLAM3::BBox2` |
| [`src/common/opencv_utils.cc`](../src/common/opencv_utils.cc) | `ORB_SLAM2::BBox2` → `ORB_SLAM3::BBox2` |
| [`ros2/oaslam_ros2_wrapper/src/oaslam_vio_node.cpp`](../ros2/oaslam_ros2_wrapper/src/oaslam_vio_node.cpp) | Removed `session_config.slam_backend.kind = oaslam::SlamBackendKind::OrbSlam3` line |
| [`src/adapters/orbslam3/internal/include/ImageDetections.h`](../src/adapters/orbslam3/internal/include/ImageDetections.h) | Added `input_width`/`input_height` params to `ObjectDetector` constructors (matching 4-param API expected by observation sources) |
| 4 ORB-SLAM3 internal files | Fixed stale `//namespace ORB_SLAM2` closing comments → `ORB_SLAM3` |

### 3.4 Remaining `Thirdparty/` Directory

The `Thirdparty/` directory at the project root still contains ORB-SLAM2's DBoW2 and g2o source trees. These are **no longer used by the build system** — ORB-SLAM3 has its own copies under `src/adapters/orbslam3/internal/Thirdparty/`. The root `Thirdparty/` directory can be safely deleted, but the `Vocabulary/` directory (also at root) is still needed for the ORB vocabulary file.

---

## 4. File Inventory

### 4.1 Modified Files

| File | Nature of Change |
|------|-----------------|
| [`include/oaslam/core/frame_packet.h`](../include/oaslam/core/frame_packet.h) | Added ImuMeasurement struct, imu_measurements vector, has_imu bool |
| [`include/oaslam/core/session_types.h`](../include/oaslam/core/session_types.h) | Added use_imu field (SlamBackendKind removed) |
| [`include/oaslam/core/tracking_types.h`](../include/oaslam/core/tracking_types.h) | Added imu_initialized, velocity_x/y/z fields |
| [`src/app/module_factories.cc`](../src/app/module_factories.cc) | Direct OrbSlam3BackendAdapter creation (no switch) |
| [`CMakeLists.txt`](../CMakeLists.txt) | ORB-SLAM3 as sole backend, no conditionals |
| [`docker/Dockerfile`](../docker/Dockerfile) | Added Sophus/message_filters; removed ORB-SLAM2 Thirdparty builds |
| [`ros2/oaslam_ros2_wrapper/CMakeLists.txt`](../ros2/oaslam_ros2_wrapper/CMakeLists.txt) | Added oaslam_vio_node target |
| [`ros2/oaslam_ros2_wrapper/package.xml`](../ros2/oaslam_ros2_wrapper/package.xml) | Added message_filters, std_msgs dependencies |
| [`src/adapters/observations/file_observation_source.h`](../src/adapters/observations/file_observation_source.h) | ORB_SLAM2 → ORB_SLAM3 namespace |
| [`src/adapters/observations/file_observation_source.cc`](../src/adapters/observations/file_observation_source.cc) | Include path + namespace → ORB_SLAM3 |
| [`src/adapters/observations/onnx_observation_source.h`](../src/adapters/observations/onnx_observation_source.h) | ORB_SLAM2 → ORB_SLAM3 namespace |
| [`src/adapters/observations/onnx_observation_source.cc`](../src/adapters/observations/onnx_observation_source.cc) | Include path + namespace → ORB_SLAM3 |
| [`src/common/opencv_utils.h`](../src/common/opencv_utils.h) | ORB_SLAM2::BBox2 → ORB_SLAM3::BBox2 |
| [`src/common/opencv_utils.cc`](../src/common/opencv_utils.cc) | ORB_SLAM2::BBox2 → ORB_SLAM3::BBox2 |

### 4.2 New Files — Adapter Layer

| File |
|------|
| [`src/adapters/orbslam3/orbslam3_backend_adapter.h`](../src/adapters/orbslam3/orbslam3_backend_adapter.h) |
| [`src/adapters/orbslam3/orbslam3_backend_adapter.cc`](../src/adapters/orbslam3/orbslam3_backend_adapter.cc) |
| [`src/adapters/orbslam3/orbslam3_pose_utils.h`](../src/adapters/orbslam3/orbslam3_pose_utils.h) |
| [`src/adapters/orbslam3/orbslam3_pose_utils.cc`](../src/adapters/orbslam3/orbslam3_pose_utils.cc) |

### 4.3 New Files — ORB-SLAM3 Internal Headers

All under `src/adapters/orbslam3/internal/include/`:

| File | Origin |
|------|--------|
| `Atlas.h` | Stock ORB-SLAM3 |
| `Camera.h` | Ported from OA-SLAM |
| `ColorManager.h` | Ported from OA-SLAM |
| `Config.h` | Stock ORB-SLAM3 |
| `Converter.h` | Stock ORB-SLAM3 |
| `Distance.h` | Ported from OA-SLAM |
| `Ellipse.h` | Ported from OA-SLAM |
| `Ellipsoid.h` | Ported from OA-SLAM |
| `Frame.h` | Stock ORB-SLAM3 |
| `FrameDrawer.h` | Stock ORB-SLAM3 |
| `G2oTypes.h` | Stock ORB-SLAM3 |
| `GeometricTools.h` | Stock ORB-SLAM3 |
| `ImageDetections.h` | Ported from OA-SLAM |
| `ImuTypes.h` | Stock ORB-SLAM3 |
| `KeyFrame.h` | Stock ORB-SLAM3 |
| `KeyFrameDatabase.h` | Stock ORB-SLAM3 |
| `Localization.h` | Ported from OA-SLAM |
| `LocalMapping.h` | Stock ORB-SLAM3 |
| `LocalObjectMapping.h` | Ported from OA-SLAM |
| `LoopClosing.h` | Stock ORB-SLAM3 |
| `Map.h` | Stock ORB-SLAM3 + MapObject patches |
| `MapDrawer.h` | Stock ORB-SLAM3 |
| `MapObject.h` | Ported from OA-SLAM |
| `MapPoint.h` | Stock ORB-SLAM3 |
| `MLPnPsolver.h` | Stock ORB-SLAM3 |
| `ObjectTrack.h` | Ported from OA-SLAM, Sophus adapted |
| `OptimizableTypes.h` | Stock ORB-SLAM3 |
| `Optimizer.h` | Stock ORB-SLAM3 |
| `OptimizerObject.h` | Ported from OA-SLAM |
| `ORBextractor.h` | Stock ORB-SLAM3 |
| `ORBmatcher.h` | Stock ORB-SLAM3 + SearchByProjectionAllPoints |
| `ORBVocabulary.h` | Stock ORB-SLAM3 |
| `p3p.h` | Ported from OA-SLAM |
| `Reconstruction.h` | Ported from OA-SLAM |
| `RingBuffer.h` | Ported from OA-SLAM |
| `SerializationUtils.h` | Stock ORB-SLAM3 |
| `Settings.h` | Stock ORB-SLAM3 |
| `Sim3Solver.h` | Stock ORB-SLAM3 |
| `System.h` | Stock ORB-SLAM3 + OA-SLAM patches |
| `Tracking.h` | Stock ORB-SLAM3 + OA-SLAM patches |
| `TwoViewReconstruction.h` | Stock ORB-SLAM3 |
| `Utils.h` | Ported from OA-SLAM |
| `Viewer.h` | Stock ORB-SLAM3 |
| `CameraModels/GeometricCamera.h` | Stock ORB-SLAM3 |
| `CameraModels/KannalaBrandt8.h` | Stock ORB-SLAM3 |
| `CameraModels/Pinhole.h` | Stock ORB-SLAM3 |

### 4.4 New Files — ORB-SLAM3 Internal Sources

All under `src/adapters/orbslam3/internal/src/`:

| File | Origin |
|------|--------|
| `Atlas.cc` | Stock ORB-SLAM3 |
| `Camera.cc` | Ported from OA-SLAM |
| `ColorManager.cc` | Ported from OA-SLAM |
| `Config.cc` | Stock ORB-SLAM3 |
| `Converter.cc` | Stock ORB-SLAM3 |
| `Distance.cc` | Ported from OA-SLAM |
| `Ellipse.cc` | Ported from OA-SLAM |
| `Ellipsoid.cc` | Ported from OA-SLAM |
| `Frame.cc` | Stock ORB-SLAM3 |
| `FrameDrawer.cc` | Stock ORB-SLAM3 |
| `G2oTypes.cc` | Stock ORB-SLAM3 |
| `GeometricTools.cc` | Stock ORB-SLAM3 |
| `ImageDetections.cc` | Ported from OA-SLAM |
| `ImuTypes.cc` | Stock ORB-SLAM3 |
| `KeyFrame.cc` | Stock ORB-SLAM3 |
| `KeyFrameDatabase.cc` | Stock ORB-SLAM3 |
| `Localization.cc` | Ported from OA-SLAM |
| `LocalMapping.cc` | Stock ORB-SLAM3 |
| `LocalObjectMapping.cc` | Ported from OA-SLAM |
| `LoopClosing.cc` | Stock ORB-SLAM3 |
| `Map.cc` | Stock ORB-SLAM3 + MapObject patches |
| `MapDrawer.cc` | Stock ORB-SLAM3 |
| `MapObject.cc` | Ported from OA-SLAM |
| `MapPoint.cc` | Stock ORB-SLAM3 |
| `MLPnPsolver.cpp` | Stock ORB-SLAM3 |
| `ObjectTrack.cc` | Ported from OA-SLAM, Sophus adapted |
| `OptimizableTypes.cpp` | Stock ORB-SLAM3 |
| `Optimizer.cc` | Stock ORB-SLAM3 |
| `OptimizerObject.cc` | Ported from OA-SLAM |
| `ORBextractor.cc` | Stock ORB-SLAM3 |
| `ORBmatcher.cc` | Stock ORB-SLAM3 + SearchByProjectionAllPoints |
| `p3p.cc` | Ported from OA-SLAM |
| `Reconstruction.cc` | Ported from OA-SLAM |
| `Settings.cc` | Stock ORB-SLAM3 |
| `Sim3Solver.cc` | Stock ORB-SLAM3 |
| `System.cc` | Stock ORB-SLAM3 + OA-SLAM patches |
| `Tracking.cc` | Stock ORB-SLAM3 + OA-SLAM patches |
| `TwoViewReconstruction.cc` | Stock ORB-SLAM3 |
| `Utils.cc` | Ported from OA-SLAM |
| `Viewer.cc` | Stock ORB-SLAM3 |
| `CameraModels/KannalaBrandt8.cpp` | Stock ORB-SLAM3 |
| `CameraModels/Pinhole.cpp` | Stock ORB-SLAM3 |

### 4.5 New Files — ORB-SLAM3 Thirdparty

| Directory | Contents |
|-----------|----------|
| `src/adapters/orbslam3/internal/Thirdparty/Sophus/` | Bundled Sophus header-only library (SE3, SO3, etc.) |
| `src/adapters/orbslam3/internal/Thirdparty/g2o/` | ORB-SLAM3 bundled g2o with types, core, solvers, stuff |
| `src/adapters/orbslam3/internal/Thirdparty/DBoW2/` | ORB-SLAM3 bundled DBoW2 with BowVector, FeatureVector, FORB |

### 4.6 New Files — ROS2 VIO Node

| File |
|------|
| [`ros2/oaslam_ros2_wrapper/src/imu_buffer.h`](../ros2/oaslam_ros2_wrapper/src/imu_buffer.h) |
| [`ros2/oaslam_ros2_wrapper/src/oaslam_vio_node.cpp`](../ros2/oaslam_ros2_wrapper/src/oaslam_vio_node.cpp) |
| [`ros2/oaslam_ros2_wrapper/launch/oaslam_vio.launch.py`](../ros2/oaslam_ros2_wrapper/launch/oaslam_vio.launch.py) |
| [`ros2/oaslam_ros2_wrapper/config/wrapper_vio.yaml`](../ros2/oaslam_ros2_wrapper/config/wrapper_vio.yaml) |
| [`ros2/oaslam_ros2_wrapper/config/d435i_imu_rgbd.yaml`](../ros2/oaslam_ros2_wrapper/config/d435i_imu_rgbd.yaml) |

---

## 5. Key Technical Decisions

### 5.1 Pose Conversion: Sophus::SE3f → Transform4d

ORB-SLAM3 returns `Sophus::SE3f` in camera-to-world convention (Tcw). The OA-SLAM pipeline uses `Transform4d` (alias for `cv::Matx44d`) in world-to-camera convention (Twc). The conversion in [`orbslam3_pose_utils.h`](../src/adapters/orbslam3/orbslam3_pose_utils.h:26) inverts Tcw→Twc and casts float→double.

### 5.2 MapPoint Positions: Eigen::Vector3f

ORB-SLAM3 `MapPoint::GetWorldPos()` returns `Eigen::Vector3f` instead of `cv::Mat`. This simplifies the OA-SLAM patches since the ellipsoid math already uses Eigen internally — a simple `.cast<double>()` suffices.

### 5.3 Atlas→GetCurrentMap() for MapObject Access

ORB-SLAM3 replaces the single `Map*` with an `Atlas` managing multiple maps. All MapObject operations (add, erase, get) go through `mpAtlas->GetCurrentMap()` instead of the direct `mpMap->` pointer used in the original OA-SLAM code.

### 5.4 Static g2o/DBoW2 Libraries

The build system creates static library targets `orbslam3_g2o` and `orbslam3_DBoW2` from ORB-SLAM3's bundled third-party sources. These are the only g2o/DBoW2 in the project (the old ORB-SLAM2 shared libs have been removed).

### 5.5 IMU Data Flow Through FramePacket

IMU measurements flow through the existing `FramePacket` struct (added `imu_measurements` vector). The `ISlamBackend::processFrame()` signature is unchanged — the backend uses IMU data when `has_imu` is true and `use_imu` is configured.

### 5.6 Hungarian Algorithm (dlib) for Detection-to-Track Matching

The object tracking block uses the Hungarian algorithm from dlib for optimal assignment of 2D detections to existing object tracks, using a cost matrix based on 2D IoU, 3D projected IoU, and map point overlap.

### 5.7 message_filters::ApproximateTimeSynchronizer for RGB+Depth Sync

The VIO node uses `message_filters` to synchronize RGB and Depth images arriving at ~30Hz, while IMU data at ~200Hz is buffered independently and drained up to the image timestamp on each synchronized callback.

---

## 6. Known Limitations and Future Work

### 6.1 Calibration

- **IMU-camera extrinsic calibration** (`IMU.T_b_c1` in [`d435i_imu_rgbd.yaml`](../ros2/oaslam_ros2_wrapper/config/d435i_imu_rgbd.yaml)) uses factory default approximate values. Production use requires Kalibr calibration with a checkerboard/AprilTag target.
- **IMU noise parameters** (gyro/accel noise density and random walk) are from the BMI055 datasheet. Allan variance calibration would improve these.

### 6.2 Implementation TODOs

- `imu_initialized` field in [`TrackingResult`](../include/oaslam/core/tracking_types.h:35) has a TODO — needs ORB-SLAM3 API exposure to query IMU preintegration convergence state.
- Object tracking in RGBD mode needs testing — current patches were primarily developed and tested with monocular mode in the original OA-SLAM.
- Depth-enhanced ellipsoid reconstruction not yet implemented (T9.3 from original plan) — RGBD depth data could improve object 3D bounding box initialization.

### 6.3 Testing Gaps

- **Unit tests** not yet written — pose conversion roundtrip, IMU measurement conversion, detection conversion, config parsing (T6.1 from plan).
- **Integration tests** not yet written — RGBD-only tracking, VIO tracking, object detection passthrough, object relocalization, semantic map persistence (T6.2 from plan).
- **Trajectory evaluation** against ground truth not yet done — requires D435i rosbag with known trajectory or TUM-style ground truth (T6.3 from plan).

### 6.4 Runtime Considerations

- ORB-SLAM3 viewer integration may need adjustment for Docker/headless environments — `use_viewer: false` is the default in VIO config.
- Atlas multi-map behavior during loop closures needs verification — MapObjects must survive map merging.
- Thread safety between `LocalObjectMapping` and ORB-SLAM3's `LocalMapping` threads needs review under concurrent operation.

### 6.5 Performance

- No performance benchmarks have been run yet (tracking latency, IMU initialization time, memory usage).

### 6.6 Cleanup

- The `Thirdparty/` directory at the project root still contains ORB-SLAM2's DBoW2 and g2o source trees. These are no longer used and can be safely deleted. Only `Vocabulary/` is still needed.
- The `README.md` still references ORB-SLAM2 — should be updated to reflect the ORB-SLAM3 migration.

---

## 7. How to Build and Run

### 7.1 Native Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### 7.2 Docker Build

```bash
docker build -f docker/Dockerfile -t oaslam-vio .
```

### 7.3 Run VIO Node (live D435i)

```bash
# Terminal 1: Launch the VIO node
ros2 launch oaslam_ros2_wrapper oaslam_vio.launch.py

# Terminal 2: Launch the RealSense camera
ros2 launch realsense2_camera rs_launch.py \
  enable_gyro:=true enable_accel:=true \
  unite_imu_method:=2 \
  align_depth.enable:=true
```

### 7.4 Run VIO Node (rosbag playback)

```bash
# Terminal 1: Play the bag with clock
ros2 bag play <bag_path> --clock

# Terminal 2: Launch VIO with simulated time
ros2 launch oaslam_ros2_wrapper oaslam_vio.launch.py use_sim_time:=true
```

### 7.5 Topic Remapping

```bash
ros2 launch oaslam_ros2_wrapper oaslam_vio.launch.py \
  rgb_topic:=/my_camera/color/image_raw \
  depth_topic:=/my_camera/aligned_depth_to_color/image_raw \
  imu_topic:=/my_camera/imu
```

---

## 8. Dependency Summary

| Dependency | Version / Source | Role | New? |
|------------|-----------------|------|------|
| **Sophus** | v1.22.10 (header-only, also bundled in Thirdparty) | SE3/SO3 Lie group types used by ORB-SLAM3 | Yes |
| **message_filters** | ROS2 Humble package | RGB+Depth approximate time synchronization | Yes |
| **dlib** | System-installed | Hungarian algorithm for detection-to-track matching | Existing |
| **Eigen3** | System-installed | Linear algebra throughout | Existing |
| **OpenCV 4.6** | System-installed (CUDA-enabled in Docker) | Image processing, DNN inference | Existing |
| **Pangolin** | System-installed | 3D visualization / viewer | Existing |
| **g2o** | Bundled as `orbslam3_g2o` static lib | Graph optimization for BA and pose optimization | New target |
| **DBoW2** | Bundled as `orbslam3_DBoW2` static lib | Bag-of-words for place recognition | New target |

### CMake Targets

| Target | Type | Description |
|--------|------|-------------|
| `orbslam3_g2o` | STATIC | ORB-SLAM3 bundled g2o (30 source files) |
| `orbslam3_DBoW2` | STATIC | ORB-SLAM3 bundled DBoW2 (6 source files) |
| `oaslam_orbslam3_internal` | STATIC | ORB-SLAM3 core + OA-SLAM patches (34 .cc + 2 camera models) |
| `oaslam_adapter_orbslam3` | STATIC | Adapter layer (2 source files) |
| `oaslam_adapter_observations` | STATIC | File and ONNX observation sources (links `oaslam_orbslam3_internal`) |
| `oaslam_app` | STATIC | Application layer with module factories and session |
| `oaslam_sdk` | INTERFACE | Top-level SDK aggregating all targets |
| `oa-slam` | EXECUTABLE | Main executable |

All targets are built unconditionally — there are no optional build flags.