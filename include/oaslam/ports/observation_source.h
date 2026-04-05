#ifndef OASLAM_PORTS_OBSERVATION_SOURCE_H
#define OASLAM_PORTS_OBSERVATION_SOURCE_H

#include <vector>

#include "oaslam/core/frame_packet.h"
#include "oaslam/core/object_types.h"

namespace oaslam {

class IObservationSource {
 public:
  virtual ~IObservationSource() = default;
  virtual std::vector<Detection2D> detect(const FramePacket& frame) = 0;
  virtual void reset() = 0;
};

}  // namespace oaslam

#endif  // OASLAM_PORTS_OBSERVATION_SOURCE_H
