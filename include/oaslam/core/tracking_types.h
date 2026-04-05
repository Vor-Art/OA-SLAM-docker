#ifndef OASLAM_CORE_TRACKING_TYPES_H
#define OASLAM_CORE_TRACKING_TYPES_H

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "oaslam/core/geometry_types.h"
#include "oaslam/core/session_types.h"

namespace oaslam {

struct TrackedFeature {
  cv::KeyPoint keypoint;
  cv::Point3d world_point;
  bool inlier = false;
};

struct SceneSlice {
  bool has_pose = false;
  Transform4d T_world_camera = Transform4d::eye();
  double mean_depth = 0.0;
  std::vector<TrackedFeature> tracked_features;
  std::vector<cv::Point3d> visible_map_points;
};

struct TrackingResult {
  TrackingState state = TrackingState::Bootstrapping;
  bool has_pose = false;
  Transform4d T_world_camera = Transform4d::eye();
  SceneSlice scene;
  double relocalization_duration_ms = -1.0;
  bool relocalization_success = false;
};

}  // namespace oaslam

#endif  // OASLAM_CORE_TRACKING_TYPES_H
