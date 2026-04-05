#include "src/adapters/orbslam2/orbslam2_backend_adapter.h"

#include <algorithm>
#include <memory>
#include <utility>

#include "src/adapters/orbslam2/internal/include/MapPoint.h"
#include "src/adapters/orbslam2/internal/include/System.h"
#include "src/common/opencv_utils.h"

namespace oaslam {

namespace {

ORB_SLAM2::enumRelocalizationMode ToLegacyRelocalizationMode(RelocalizationMode mode) {
  switch (mode) {
    case RelocalizationMode::Objects:
      return ORB_SLAM2::RELOC_OBJECTS;
    case RelocalizationMode::PointsAndObjects:
      return ORB_SLAM2::RELOC_OBJECTS_POINTS;
    case RelocalizationMode::Points:
    default:
      return ORB_SLAM2::RELOC_POINTS;
  }
}

}  // namespace

OrbSlam2BackendAdapter::OrbSlam2BackendAdapter(const SlamBackendConfig& config)
    : config_(config) {
  ensureLegacyRuntime();
  legacy_system_->SetRelocalizationMode(ToLegacyRelocalizationMode(config_.relocalization_mode));
}

OrbSlam2BackendAdapter::~OrbSlam2BackendAdapter() {
  shutdown();
}

void OrbSlam2BackendAdapter::ensureLegacyRuntime() const {
  if (legacy_system_) {
    return;
  }

  legacy_system_ = std::make_unique<ORB_SLAM2::System>(
      config_.vocabulary_file, config_.camera_settings_file, ORB_SLAM2::System::MONOCULAR,
      config_.use_viewer, config_.use_ar_viewer, config_.use_objects_in_local_ba);
  legacy_system_->SetRelocalizationMode(ToLegacyRelocalizationMode(config_.relocalization_mode));
}

std::vector<std::shared_ptr<ORB_SLAM2::Detection>> OrbSlam2BackendAdapter::toLegacyDetections(
    const std::vector<Detection2D>& detections) const {
  std::vector<std::shared_ptr<ORB_SLAM2::Detection>> legacy_detections;
  legacy_detections.reserve(detections.size());
  for (const auto& detection : detections) {
    legacy_detections.push_back(std::make_shared<ORB_SLAM2::Detection>(
        detection.category_id, detection.score, RectToEigenBBox(detection.bbox)));
  }
  return legacy_detections;
}

TrackingState OrbSlam2BackendAdapter::mapTrackingState(int legacy_state, bool relocalizing) const {
  if (relocalizing) {
    return TrackingState::Relocalizing;
  }

  switch (legacy_state) {
    case ORB_SLAM2::Tracking::OK:
      return TrackingState::Tracking;
    case ORB_SLAM2::Tracking::LOST:
      return TrackingState::Lost;
    case ORB_SLAM2::Tracking::NOT_INITIALIZED:
    case ORB_SLAM2::Tracking::NO_IMAGES_YET:
    default:
      return TrackingState::Bootstrapping;
  }
}

TrackingResult OrbSlam2BackendAdapter::processFrame(
    const FramePacket& frame, const std::vector<Detection2D>& detections,
    const std::vector<PoseProposal>& pose_proposals) {
  (void)pose_proposals;

  ensureLegacyRuntime();
  const auto legacy_detections = toLegacyDetections(detections);
  const cv::Mat pose = legacy_system_->TrackMonocular(frame.image, frame.timestamp, legacy_detections, false);

  TrackingResult result;
  result.relocalization_duration_ms = legacy_system_->relocalization_duration;
  result.relocalization_success = legacy_system_->relocalization_status;
  result.state = mapTrackingState(legacy_system_->GetTrackingState(), false);

  if (!pose.empty()) {
    result.has_pose = true;
    result.T_world_camera = CvMatToTransform(pose);
    result.scene.has_pose = true;
    result.scene.T_world_camera = result.T_world_camera;
  }

  const auto tracked_points = legacy_system_->GetTrackedMapPoints();
  const auto tracked_keypoints = legacy_system_->GetTrackedKeyPointsUn();
  const std::size_t count = std::min(tracked_points.size(), tracked_keypoints.size());
  result.scene.tracked_features.reserve(count);
  result.scene.visible_map_points.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    ORB_SLAM2::MapPoint* map_point = tracked_points[i];
    if (map_point == nullptr) {
      continue;
    }

    const cv::Mat position = map_point->GetWorldPos();
    const cv::Point3d world_point(position.at<float>(0), position.at<float>(1),
                                  position.at<float>(2));
    result.scene.visible_map_points.push_back(world_point);
    result.scene.tracked_features.push_back(
        TrackedFeature{tracked_keypoints[i], world_point, true});
  }

  if (!pose.empty()) {
    result.scene.mean_depth = EstimateMeanDepthCameraZ(pose, result.scene.visible_map_points);
  }

  return result;
}

void OrbSlam2BackendAdapter::reset() {
  ensureLegacyRuntime();
  legacy_system_->Reset();
}

void OrbSlam2BackendAdapter::shutdown() {
  if (is_shutdown_) {
    return;
  }

  if (legacy_system_) {
    legacy_system_->Shutdown();
  }
  is_shutdown_ = true;
}

bool OrbSlam2BackendAdapter::shouldQuit() const {
  return legacy_system_ && legacy_system_->ShouldQuit();
}

}  // namespace oaslam
