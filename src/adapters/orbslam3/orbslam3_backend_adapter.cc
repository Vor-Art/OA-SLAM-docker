#include "src/adapters/orbslam3/orbslam3_backend_adapter.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "src/adapters/orbslam3/internal/include/MapPoint.h"
#include "src/adapters/orbslam3/internal/include/System.h"
#include "src/adapters/orbslam3/internal/include/Tracking.h"
#include "src/adapters/orbslam3/orbslam3_pose_utils.h"

namespace oaslam {

OrbSlam3BackendAdapter::OrbSlam3BackendAdapter(const std::string& vocab_path,
                                               const std::string& settings_path,
                                               bool use_imu,
                                               bool use_viewer)
    : use_imu_(use_imu) {
  const auto sensor_type = use_imu ? ORB_SLAM3::System::IMU_RGBD
                                   : ORB_SLAM3::System::RGBD;
  system_ = std::make_unique<ORB_SLAM3::System>(
      vocab_path, settings_path, sensor_type, use_viewer);
}

OrbSlam3BackendAdapter::~OrbSlam3BackendAdapter() {
  shutdown();
}

TrackingResult OrbSlam3BackendAdapter::processFrame(
    const FramePacket& frame,
    const std::vector<Detection2D>& detections,
    const std::vector<PoseProposal>& pose_proposals) {
  (void)detections;
  (void)pose_proposals;

  TrackingResult result;

  // Call ORB-SLAM3 TrackRGBD with or without IMU data
  Sophus::SE3f Tcw;
  if (use_imu_ && frame.has_imu) {
    auto imu_points =
        orbslam3_utils::ToImuPoints(frame.imu_measurements);
    Tcw = system_->TrackRGBD(frame.image, frame.depth_image,
                             frame.timestamp, imu_points);
  } else {
    Tcw = system_->TrackRGBD(frame.image, frame.depth_image,
                             frame.timestamp);
  }

  // Map the ORB-SLAM3 tracking state to our TrackingState enum
  const int orbslam3_state = system_->GetTrackingState();
  result.state = mapTrackingState(orbslam3_state);

  // Convert pose if tracking is OK
  if (orbslam3_utils::IsValidPose(Tcw) &&
      result.state == TrackingState::Tracking) {
    result.has_pose = true;
    result.T_world_camera = orbslam3_utils::SophusTcwToTransform4d(Tcw);
    result.scene.has_pose = true;
    result.scene.T_world_camera = result.T_world_camera;
  }

  // Extract tracked map points and keypoints
  const auto tracked_points = system_->GetTrackedMapPoints();
  const auto tracked_keypoints = system_->GetTrackedKeyPointsUn();
  const std::size_t count =
      std::min(tracked_points.size(), tracked_keypoints.size());
  result.scene.tracked_features.reserve(count);
  result.scene.visible_map_points.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    ORB_SLAM3::MapPoint* map_point = tracked_points[i];
    if (map_point == nullptr || map_point->isBad()) {
      continue;
    }

    const Eigen::Vector3f position = map_point->GetWorldPos();
    const cv::Point3d world_point = orbslam3_utils::EigenToPoint3d(position);
    result.scene.visible_map_points.push_back(world_point);
    result.scene.tracked_features.push_back(
        TrackedFeature{tracked_keypoints[i], world_point, true});
  }

  // Estimate mean depth if we have a valid pose and map points
  if (result.has_pose && !result.scene.visible_map_points.empty()) {
    // Compute mean depth in camera Z direction
    const Transform4d& Twc = result.T_world_camera;
    // Tcw = Twc^-1, but we can compute camera-frame Z directly
    // For each world point p, camera_z = R_cw.row(2) * p + t_cw.z
    // where R_cw, t_cw come from Tcw (not Twc)
    Eigen::Matrix4f Tcw_mat = Tcw.matrix();
    double depth_sum = 0.0;
    int depth_count = 0;
    for (const auto& wp : result.scene.visible_map_points) {
      double cz = static_cast<double>(Tcw_mat(2, 0)) * wp.x +
                  static_cast<double>(Tcw_mat(2, 1)) * wp.y +
                  static_cast<double>(Tcw_mat(2, 2)) * wp.z +
                  static_cast<double>(Tcw_mat(2, 3));
      if (cz > 0.0) {
        depth_sum += cz;
        ++depth_count;
      }
    }
    if (depth_count > 0) {
      result.scene.mean_depth = depth_sum / depth_count;
    }
  }

  // TODO(T2.6): Check IMU initialization status when ORB-SLAM3 exposes
  // isImuInitialized() or equivalent API. For now, approximate by checking
  // if the system has been running long enough with IMU data.
  result.imu_initialized = false;

  return result;
}

void OrbSlam3BackendAdapter::reset() {
  if (system_) {
    system_->ResetActiveMap();
  }
}

void OrbSlam3BackendAdapter::shutdown() {
  if (shutdown_called_) {
    return;
  }
  if (system_) {
    system_->Shutdown();
  }
  shutdown_called_ = true;
}

bool OrbSlam3BackendAdapter::shouldQuit() const {
  return system_ && system_->isShutDown();
}

int OrbSlam3BackendAdapter::keyframesInQueue() const {
  if (system_) {
    return system_->GetLocalMappingKeyframesInQueue();
  }
  return 0;
}

TrackingState OrbSlam3BackendAdapter::mapTrackingState(
    int orbslam3_state) const {
  // ORB-SLAM3 Tracking::eTrackingState:
  //   SYSTEM_NOT_READY = -1
  //   NO_IMAGES_YET    =  0
  //   NOT_INITIALIZED  =  1
  //   OK               =  2
  //   RECENTLY_LOST    =  3
  //   LOST             =  4
  //   OK_KLT           =  5
  switch (orbslam3_state) {
    case ORB_SLAM3::Tracking::OK:
    case ORB_SLAM3::Tracking::OK_KLT:
      return TrackingState::Tracking;
    case ORB_SLAM3::Tracking::RECENTLY_LOST:
    case ORB_SLAM3::Tracking::LOST:
      return TrackingState::Lost;
    case ORB_SLAM3::Tracking::NOT_INITIALIZED:
    case ORB_SLAM3::Tracking::NO_IMAGES_YET:
    case ORB_SLAM3::Tracking::SYSTEM_NOT_READY:
    default:
      return TrackingState::Bootstrapping;
  }
}

}  // namespace oaslam
