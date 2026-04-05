#ifndef OASLAM_CORE_OBJECT_TYPES_H
#define OASLAM_CORE_OBJECT_TYPES_H

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "oaslam/core/geometry_types.h"

namespace oaslam {

struct Detection2D {
  uint32_t category_id = 0;
  float score = 0.0F;
  cv::Rect2d bbox;
};

struct PoseProposal {
  Transform4d T_world_camera = Transform4d::eye();
  float confidence = 0.0F;
  std::string source;
};

struct ObjectPreTrack {
  std::vector<PoseProposal> pose_proposals;
};

struct ObjectStateView {
  std::vector<Detection2D> detections;
  std::vector<cv::RotatedRect> object_projections;
};

}  // namespace oaslam

#endif  // OASLAM_CORE_OBJECT_TYPES_H
