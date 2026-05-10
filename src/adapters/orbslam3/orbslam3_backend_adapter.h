#ifndef OASLAM_ADAPTERS_ORBSLAM3_ORBSLAM3_BACKEND_ADAPTER_H
#define OASLAM_ADAPTERS_ORBSLAM3_ORBSLAM3_BACKEND_ADAPTER_H

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "oaslam/core/semantic_map_types.h"
#include "oaslam/core/session_types.h"
#include "oaslam/ports/slam_backend.h"

// Forward declarations — do NOT include ORB-SLAM3 headers here
namespace ORB_SLAM3 {
class System;
class Detection;
}

namespace oaslam {

class OrbSlam3BackendAdapter final : public ISlamBackend {
 public:
  /// Constructor takes the full SlamBackendConfig.
  /// vocab_path: path to ORBvoc.txt (from config.vocabulary_file)
  /// settings_path: path to the YAML settings file (from config.camera_settings_file)
  /// use_imu: whether to use IMU_RGBD mode (true) or plain RGBD mode (false)
  /// use_viewer: whether to enable the ORB-SLAM3 viewer
  OrbSlam3BackendAdapter(const std::string& vocab_path,
                         const std::string& settings_path,
                         bool use_imu,
                         bool use_viewer);
  ~OrbSlam3BackendAdapter() override;

  TrackingResult processFrame(
      const FramePacket& frame,
      const std::vector<Detection2D>& detections,
      const std::vector<PoseProposal>& pose_proposals) override;
  void reset() override;
  void shutdown() override;
  bool saveFinalTrajectory(const std::string& frame_trajectory_path,
                           const std::string& keyframe_trajectory_path) override;
  bool saveObjectMap(const std::string& object_map_txt_path,
                     const std::string& object_map_obj_path) override;
  int keyframesInQueue() const override;

  bool shouldQuit() const;

 private:
  TrackingState mapTrackingState(int orbslam3_state) const;
  std::vector<std::shared_ptr<ORB_SLAM3::Detection>> toLegacyDetections(
      const std::vector<Detection2D>& detections) const;

  std::unique_ptr<ORB_SLAM3::System> system_;
  std::unordered_map<unsigned int, SemanticObject> last_semantic_objects_;
  std::unordered_set<unsigned long> seen_map_point_ids_;
  unsigned long last_semantic_map_id_ = 0;
  std::uint64_t semantic_sequence_ = 0;
  bool use_imu_ = false;
  bool shutdown_called_ = false;
};

}  // namespace oaslam

#endif  // OASLAM_ADAPTERS_ORBSLAM3_ORBSLAM3_BACKEND_ADAPTER_H
