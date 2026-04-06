#ifndef OASLAM_CORE_SESSION_TYPES_H
#define OASLAM_CORE_SESSION_TYPES_H

#include <string>
#include <vector>

namespace oaslam {

enum class TrackingState { Bootstrapping, Tracking, Relocalizing, Lost };
enum class RelocalizationMode { Points, Objects, PointsAndObjects };
enum class ObservationSourceKind { None, File, Onnx };

struct ModelInputSize {
  int width = 0;
  int height = 0;
};

struct SlamBackendConfig {
  std::string vocabulary_file;
  std::string camera_settings_file;
  bool use_viewer = true;
  bool use_ar_viewer = false;
  int use_objects_in_local_ba = 0;
  RelocalizationMode relocalization_mode = RelocalizationMode::Points;
  bool use_imu = false;
};

struct ObservationSourceConfig {
  ObservationSourceKind kind = ObservationSourceKind::None;
  std::string source_path;
  std::vector<int> ignored_categories;
  int model_input_width = 0;
  int model_input_height = 0;
};

struct VisualizerConfig {
  bool enabled = true;
};

struct AgentGatewayConfig {
  bool enabled = true;
};

struct SessionConfig {
  SlamBackendConfig slam_backend;
  ObservationSourceConfig observation_source;
  VisualizerConfig visualizer;
  AgentGatewayConfig agent_gateway;
};

ModelInputSize LoadModelInputSize(const std::string& camera_config_path);

}  // namespace oaslam

#endif  // OASLAM_CORE_SESSION_TYPES_H
