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

 private:
  SessionConfig config_;
  ModuleBundle modules_;
  bool is_shutdown_ = false;
};

}  // namespace oaslam

#endif  // OASLAM_APP_SLAM_SESSION_H
