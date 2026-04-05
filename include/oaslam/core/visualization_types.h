#ifndef OASLAM_CORE_VISUALIZATION_TYPES_H
#define OASLAM_CORE_VISUALIZATION_TYPES_H

#include "oaslam/core/frame_packet.h"
#include "oaslam/core/object_types.h"
#include "oaslam/core/tracking_types.h"

namespace oaslam {

struct VisualizationFrame {
  FramePacket frame;
  TrackingResult tracking;
  ObjectStateView objects;
};

struct SessionFrameResult {
  TrackingResult tracking;
  ObjectStateView objects;
  bool quit_requested = false;
};

}  // namespace oaslam

#endif  // OASLAM_CORE_VISUALIZATION_TYPES_H
