#ifndef OASLAM_PORTS_OBJECT_PIPELINE_H
#define OASLAM_PORTS_OBJECT_PIPELINE_H

#include <filesystem>
#include <vector>

#include "oaslam/core/frame_packet.h"
#include "oaslam/core/object_types.h"
#include "oaslam/core/session_types.h"
#include "oaslam/core/tracking_types.h"

namespace oaslam {

class IObjectPipeline {
 public:
  virtual ~IObjectPipeline() = default;
  virtual ObjectPreTrack preTrack(const FramePacket& frame,
                                  const std::vector<Detection2D>& detections) = 0;
  virtual ObjectStateView postTrack(const FramePacket& frame,
                                    const std::vector<Detection2D>& detections,
                                    const TrackingResult& tracking) = 0;
  virtual void setMode(SessionMode mode) = 0;
  virtual void reset() = 0;
  virtual void loadState(const std::filesystem::path& root) = 0;
  virtual void saveState(const std::filesystem::path& root) const = 0;
};

}  // namespace oaslam

#endif  // OASLAM_PORTS_OBJECT_PIPELINE_H
