#ifndef OASLAM_PORTS_SLAM_BACKEND_H
#define OASLAM_PORTS_SLAM_BACKEND_H

#include <string>
#include <vector>

#include "oaslam/core/frame_packet.h"
#include "oaslam/core/object_types.h"
#include "oaslam/core/session_types.h"
#include "oaslam/core/tracking_types.h"

namespace oaslam {

class ISlamBackend {
 public:
  virtual ~ISlamBackend() = default;
  virtual TrackingResult processFrame(
      const FramePacket& frame, const std::vector<Detection2D>& detections,
      const std::vector<PoseProposal>& pose_proposals) = 0;
  virtual void reset() = 0;
  virtual void shutdown() = 0;
  virtual bool saveFinalTrajectory(const std::string& frame_trajectory_path,
                                   const std::string& keyframe_trajectory_path) {
    (void)frame_trajectory_path;
    (void)keyframe_trajectory_path;
    return false;
  }

  /// Returns the number of keyframes waiting in the local mapping queue.
  /// Used for backpressure: callers can throttle frame feeding when the queue
  /// is deep. Default returns 0 (no backpressure information available).
  virtual int keyframesInQueue() const { return 0; }
};

}  // namespace oaslam

#endif  // OASLAM_PORTS_SLAM_BACKEND_H
