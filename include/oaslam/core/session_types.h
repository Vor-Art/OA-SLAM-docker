#ifndef OASLAM_CORE_SESSION_TYPES_H
#define OASLAM_CORE_SESSION_TYPES_H

#include <string>
#include <vector>

namespace oaslam {

enum class TrackingState { Bootstrapping, Tracking, Relocalizing, Lost };
enum class RelocalizationMode { Points, Objects, PointsAndObjects };
enum class ObservationSourceKind { None, File, Onnx };

struct SlamBackendConfig {
  std::string vocabulary_file;
  std::string camera_settings_file;
  bool use_viewer = true;
  bool use_ar_viewer = false;
  int use_objects_in_local_ba = 0;
  RelocalizationMode relocalization_mode = RelocalizationMode::Points;
};

struct ObservationSourceConfig {
  ObservationSourceKind kind = ObservationSourceKind::None;
  std::string source_path;
  std::vector<int> ignored_categories;
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

}  // namespace oaslam

#endif  // OASLAM_CORE_SESSION_TYPES_H
