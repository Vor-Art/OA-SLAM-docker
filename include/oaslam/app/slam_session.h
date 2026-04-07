#ifndef OASLAM_APP_SLAM_SESSION_H
#define OASLAM_APP_SLAM_SESSION_H

#include "oaslam/app/module_factories.h"
#include "oaslam/core/visualization_types.h"

namespace oaslam {

class SlamSession {
 public:
  SlamSession(SessionConfig config, ModuleBundle modules);

  SessionFrameResult processFrame(const FramePacket& frame);
  void reset();
  void shutdown();

  /// Returns the number of keyframes waiting in the SLAM backend's local
  /// mapping queue. Returns 0 if the backend does not support this query.
  int keyframesInQueue() const;

 private:
  SessionConfig config_;
  ModuleBundle modules_;
  bool is_shutdown_ = false;
};

}  // namespace oaslam

#endif  // OASLAM_APP_SLAM_SESSION_H
