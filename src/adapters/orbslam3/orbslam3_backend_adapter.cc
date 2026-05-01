#include "src/adapters/orbslam3/orbslam3_backend_adapter.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "src/adapters/orbslam3/internal/include/ImageDetections.h"
#include "src/adapters/orbslam3/internal/include/MapObject.h"
#include "src/adapters/orbslam3/internal/include/MapPoint.h"
#include "src/adapters/orbslam3/internal/include/System.h"
#include "src/adapters/orbslam3/internal/include/Tracking.h"
#include "src/adapters/orbslam3/orbslam3_pose_utils.h"

namespace oaslam {
namespace {

Transform4d MakeTransform4d(const Eigen::Matrix3d& rotation,
                            const Eigen::Vector3d& translation) {
  Transform4d transform = Transform4d::eye();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      transform(row, col) = rotation(row, col);
    }
    transform(row, 3) = translation(row);
  }
  return transform;
}

SemanticObjectStatus ToSemanticObjectStatus(
    ORB_SLAM3::ObjectTrackStatus status) {
  switch (status) {
    case ORB_SLAM3::ObjectTrackStatus::ONLY_2D:
      return SemanticObjectStatus::Only2D;
    case ORB_SLAM3::ObjectTrackStatus::INITIALIZED:
      return SemanticObjectStatus::Initialized;
    case ORB_SLAM3::ObjectTrackStatus::IN_MAP:
      return SemanticObjectStatus::InMap;
    case ORB_SLAM3::ObjectTrackStatus::BAD:
    default:
      return SemanticObjectStatus::Bad;
  }
}

SemanticObject ToSemanticObject(const ORB_SLAM3::MapObject& map_object) {
  const ORB_SLAM3::ObjectTrack* track = map_object.GetTrack();
  const ORB_SLAM3::Ellipsoid& ellipsoid = map_object.GetEllipsoid();

  SemanticObject object;
  object.local_object_id = track ? track->GetId() : 0U;
  object.category_id = track ? track->GetCategoryId() : 0U;
  object.status = track ? ToSemanticObjectStatus(track->GetStatus())
                        : SemanticObjectStatus::Bad;
  object.T_map_object =
      MakeTransform4d(ellipsoid.GetOrientation(), ellipsoid.GetCenter());

  const Eigen::Vector3d axes = ellipsoid.GetAxes();
  object.axes = cv::Vec3d(axes(0), axes(1), axes(2));
  object.observation_count = track ? track->GetNbObservations() : 0U;
  object.keyframe_observation_count =
      track ? track->GetNbObservationsInKeyFrame() : 0U;
  object.last_obs_score = track ? track->GetLastObsScore() : 0.0;
  object.last_obs_frame_id = track ? track->GetLastObsFrameId() : -1;
  return object;
}

bool NearlyEqual(double lhs, double rhs, double tolerance) {
  return std::abs(lhs - rhs) <= tolerance;
}

bool SameTransform(const Transform4d& lhs, const Transform4d& rhs,
                   double tolerance) {
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      if (!NearlyEqual(lhs(row, col), rhs(row, col), tolerance)) {
        return false;
      }
    }
  }
  return true;
}

bool SameSemanticObject(const SemanticObject& lhs, const SemanticObject& rhs) {
  if (lhs.local_object_id != rhs.local_object_id ||
      lhs.category_id != rhs.category_id || lhs.status != rhs.status ||
      lhs.observation_count != rhs.observation_count ||
      lhs.keyframe_observation_count != rhs.keyframe_observation_count ||
      lhs.last_obs_frame_id != rhs.last_obs_frame_id) {
    return false;
  }

  if (!SameTransform(lhs.T_map_object, rhs.T_map_object, 1e-5) ||
      !NearlyEqual(lhs.axes[0], rhs.axes[0], 1e-5) ||
      !NearlyEqual(lhs.axes[1], rhs.axes[1], 1e-5) ||
      !NearlyEqual(lhs.axes[2], rhs.axes[2], 1e-5) ||
      !NearlyEqual(lhs.last_obs_score, rhs.last_obs_score, 1e-6)) {
    return false;
  }

  return true;
}

}  // namespace

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

std::vector<ORB_SLAM3::Detection::Ptr> OrbSlam3BackendAdapter::toLegacyDetections(
    const std::vector<Detection2D>& detections) const {
  std::vector<ORB_SLAM3::Detection::Ptr> legacy;
  legacy.reserve(detections.size());
  for (const auto& det : detections) {
    Eigen::Vector4d bbox(det.bbox.x, det.bbox.y,
                         det.bbox.x + det.bbox.width,
                         det.bbox.y + det.bbox.height);
    legacy.push_back(std::make_shared<ORB_SLAM3::Detection>(
        det.category_id, det.score, bbox));
  }
  return legacy;
}

TrackingResult OrbSlam3BackendAdapter::processFrame(
    const FramePacket& frame,
    const std::vector<Detection2D>& detections,
    const std::vector<PoseProposal>& pose_proposals) {
  (void)pose_proposals;

  TrackingResult result;
  std::string stage = "start";

  try {
  stage = "convert-detections";
  const auto legacy_detections = toLegacyDetections(detections);

  // Call ORB-SLAM3 TrackRGBD with or without IMU data.
  Sophus::SE3f Tcw;
  if (use_imu_) {
    stage = "convert-imu";
    auto imu_points =
        orbslam3_utils::ToImuPoints(frame.imu_measurements);
    stage = "track-rgbd-imu";
    Tcw = system_->TrackRGBD(frame.image, frame.depth_image,
                             frame.timestamp, legacy_detections,
                             imu_points);
  } else {
    stage = "track-rgbd";
    Tcw = system_->TrackRGBD(frame.image, frame.depth_image,
                             frame.timestamp, legacy_detections);
  }

  // Map the ORB-SLAM3 tracking state to our TrackingState enum
  stage = "get-tracking-state";
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

  stage = "get-current-map-id";
  result.semantic_map.sequence = ++semantic_sequence_;
  result.semantic_map_delta.sequence = semantic_sequence_;
  result.semantic_map.map_id = system_->GetCurrentMapId();
  result.semantic_map_delta.map_id = result.semantic_map.map_id;

  if (result.semantic_map.map_id != last_semantic_map_id_) {
    result.semantic_map_delta.reset = true;
    last_semantic_objects_.clear();
    last_semantic_map_id_ = result.semantic_map.map_id;
  }

  std::unordered_map<unsigned int, SemanticObject> current_semantic_objects;
  stage = "get-map-objects";
  const auto map_objects = system_->GetAllMapObjects();
  result.semantic_map.objects.reserve(map_objects.size());
  for (ORB_SLAM3::MapObject* map_object : map_objects) {
    if (map_object == nullptr || map_object->GetTrack() == nullptr ||
        map_object->GetTrack()->IsBad()) {
      continue;
    }

    SemanticObject semantic_object = ToSemanticObject(*map_object);
    const unsigned int object_id = semantic_object.local_object_id;
    result.semantic_map.objects.push_back(semantic_object);
    current_semantic_objects.emplace(object_id, semantic_object);

    const auto it = last_semantic_objects_.find(object_id);
    if (it == last_semantic_objects_.end()) {
      result.semantic_map_delta.added.push_back(semantic_object);
    } else if (!SameSemanticObject(it->second, semantic_object)) {
      result.semantic_map_delta.updated.push_back(semantic_object);
    }
  }

  for (const auto& previous_entry : last_semantic_objects_) {
    if (current_semantic_objects.find(previous_entry.first) ==
        current_semantic_objects.end()) {
      result.semantic_map_delta.removed_object_ids.push_back(previous_entry.first);
    }
  }

  last_semantic_objects_ = std::move(current_semantic_objects);

  std::sort(result.semantic_map.objects.begin(), result.semantic_map.objects.end(),
            [](const SemanticObject& lhs, const SemanticObject& rhs) {
              return lhs.local_object_id < rhs.local_object_id;
            });
  std::sort(result.semantic_map_delta.added.begin(),
            result.semantic_map_delta.added.end(),
            [](const SemanticObject& lhs, const SemanticObject& rhs) {
              return lhs.local_object_id < rhs.local_object_id;
            });
  std::sort(result.semantic_map_delta.updated.begin(),
            result.semantic_map_delta.updated.end(),
            [](const SemanticObject& lhs, const SemanticObject& rhs) {
              return lhs.local_object_id < rhs.local_object_id;
            });
  std::sort(result.semantic_map_delta.removed_object_ids.begin(),
            result.semantic_map_delta.removed_object_ids.end());

  // Extract tracked map points and keypoints
  stage = "get-map-points";
  const auto map_points = system_->GetAllMapPoints();
  stage = "get-tracked-map-points";
  const auto tracked_points = system_->GetTrackedMapPoints();
  stage = "get-tracked-keypoints";
  const auto tracked_keypoints = system_->GetTrackedKeyPointsUn();
  result.scene.map_points.reserve(map_points.size());
  result.scene.new_map_points.reserve(map_points.size());
  const std::size_t count =
      std::min(tracked_points.size(), tracked_keypoints.size());
  result.scene.tracked_features.reserve(count);
  result.scene.visible_map_points.reserve(count);

  for (ORB_SLAM3::MapPoint* map_point : map_points) {
    if (map_point == nullptr || map_point->isBad()) {
      continue;
    }

    const Eigen::Vector3f position = map_point->GetWorldPos();
    const cv::Point3d world_point = orbslam3_utils::EigenToPoint3d(position);
    result.scene.map_points.push_back(world_point);
    if (seen_map_point_ids_.insert(map_point->mnId).second) {
      result.scene.new_map_points.push_back(world_point);
    }
  }

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
  } catch (const std::exception& exc) {
    std::ostringstream context;
    context << "ORB-SLAM3 backend processFrame failed"
            << " | stage=" << stage
            << " | frame=" << frame.frame_id
            << " | t=" << frame.timestamp
            << " | imu=" << frame.imu_measurements.size()
            << " | detections=" << detections.size()
            << " | err=" << exc.what();
    throw std::runtime_error(context.str());
  }

  return result;
}

void OrbSlam3BackendAdapter::reset() {
  if (system_) {
    system_->ResetActiveMap();
  }
  last_semantic_objects_.clear();
  last_semantic_map_id_ = 0;
  semantic_sequence_ = 0;
  seen_map_point_ids_.clear();
}

void OrbSlam3BackendAdapter::shutdown() {
  if (shutdown_called_) {
    return;
  }
  shutdown_called_ = true;
  if (system_) {
    try {
      system_->Shutdown();
    } catch (const std::exception& exc) {
      std::cerr << "ORB-SLAM3 shutdown warning: " << exc.what() << std::endl;
    } catch (...) {
      std::cerr << "ORB-SLAM3 shutdown warning: unknown exception" << std::endl;
    }
  }
  last_semantic_objects_.clear();
  last_semantic_map_id_ = 0;
  semantic_sequence_ = 0;
  seen_map_point_ids_.clear();
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
