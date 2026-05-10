#ifndef OASLAM_APP_SLAM_SESSION_H
#define OASLAM_APP_SLAM_SESSION_H

#include <string>

#include "oaslam/app/module_factories.h"
#include "oaslam/core/visualization_types.h"

namespace oaslam {

class SlamSession {
 public:
  SlamSession(SessionConfig config, ModuleBundle modules);

  SessionFrameResult processFrame(const FramePacket& frame);
  void reset();
  void shutdown();
  bool saveFinalTrajectory(const std::string& frame_trajectory_path,
                           const std::string& keyframe_trajectory_path);
  bool saveObjectMap(const std::string& object_map_txt_path,
                     const std::string& object_map_obj_path);

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
