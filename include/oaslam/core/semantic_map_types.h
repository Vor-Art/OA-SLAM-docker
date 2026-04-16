#ifndef OASLAM_CORE_SEMANTIC_MAP_TYPES_H
#define OASLAM_CORE_SEMANTIC_MAP_TYPES_H

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "oaslam/core/geometry_types.h"

namespace oaslam {

enum class SemanticObjectStatus : std::uint8_t {
  Only2D = 0,
  Initialized = 1,
  InMap = 2,
  Bad = 3,
};

struct SemanticObject {
  std::uint32_t local_object_id = 0;
  std::uint32_t category_id = 0;
  SemanticObjectStatus status = SemanticObjectStatus::Only2D;
  Transform4d T_map_object = Transform4d::eye();
  cv::Vec3d axes{0.0, 0.0, 0.0};
  std::uint32_t observation_count = 0;
  std::uint32_t keyframe_observation_count = 0;
  double last_obs_score = 0.0;
  std::int32_t last_obs_frame_id = -1;
};

struct SemanticMapSnapshot {
  std::uint64_t sequence = 0;
  std::uint64_t map_id = 0;
  std::vector<SemanticObject> objects;
};

struct SemanticMapDelta {
  std::uint64_t sequence = 0;
  std::uint64_t map_id = 0;
  bool reset = false;
  std::vector<SemanticObject> added;
  std::vector<SemanticObject> updated;
  std::vector<std::uint32_t> removed_object_ids;

  bool empty() const {
    return !reset && added.empty() && updated.empty() &&
           removed_object_ids.empty();
  }
};

}  // namespace oaslam

#endif  // OASLAM_CORE_SEMANTIC_MAP_TYPES_H
