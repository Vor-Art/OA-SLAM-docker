#ifndef OASLAM_ADAPTERS_ORBSLAM2_ORBSLAM2_BACKEND_ADAPTER_H
#define OASLAM_ADAPTERS_ORBSLAM2_ORBSLAM2_BACKEND_ADAPTER_H

#include <memory>
#include <vector>

#include "oaslam/core/session_types.h"
#include "oaslam/ports/slam_backend.h"

namespace ORB_SLAM2 {
class System;
class Detection;
}

namespace oaslam {

class OrbSlam2BackendAdapter : public ISlamBackend {
 public:
  explicit OrbSlam2BackendAdapter(const SlamBackendConfig& config);
  ~OrbSlam2BackendAdapter() override;

  TrackingResult processFrame(const FramePacket& frame,
                              const std::vector<Detection2D>& detections,
                              const std::vector<PoseProposal>& pose_proposals) override;
  void reset() override;
  void shutdown() override;

  bool shouldQuit() const;

 private:
  void ensureLegacyRuntime() const;
  std::vector<std::shared_ptr<ORB_SLAM2::Detection>> toLegacyDetections(
      const std::vector<Detection2D>& detections) const;
  TrackingState mapTrackingState(int legacy_state, bool relocalizing) const;

  SlamBackendConfig config_;
  mutable std::unique_ptr<ORB_SLAM2::System> legacy_system_;
  mutable bool is_shutdown_ = false;
};

}  // namespace oaslam

#endif  // OASLAM_ADAPTERS_ORBSLAM2_ORBSLAM2_BACKEND_ADAPTER_H
