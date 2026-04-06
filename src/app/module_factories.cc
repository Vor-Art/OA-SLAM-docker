#include "oaslam/app/module_factories.h"

#include <functional>
#include <memory>

#include "src/adapters/agent/null_agent_gateway.h"
#include "src/adapters/oa_objects/oa_object_pipeline_adapter.h"
#include "src/adapters/observations/file_observation_source.h"
#include "src/adapters/observations/onnx_observation_source.h"
#include "src/adapters/orbslam3/orbslam3_backend_adapter.h"
#include "src/adapters/visualization/pangolin_visualizer.h"

namespace oaslam {

namespace {

class NullObservationSource : public IObservationSource {
 public:
  std::vector<Detection2D> detect(const FramePacket& frame) override {
    (void)frame;
    return {};
  }

  void reset() override {}
};

}  // namespace

ModuleBundle CreateDefaultModules(const SessionConfig& config) {
  ModuleBundle bundle;

  std::function<bool()> quit_check = []() { return false; };

  {
    auto backend = std::make_unique<OrbSlam3BackendAdapter>(
        config.slam_backend.vocabulary_file,
        config.slam_backend.camera_settings_file,
        config.slam_backend.use_imu,
        config.slam_backend.use_viewer);
    auto* ptr = backend.get();
    quit_check = [ptr]() { return ptr->shouldQuit(); };
    bundle.slam_backend = std::move(backend);
  }

  switch (config.observation_source.kind) {
    case ObservationSourceKind::File:
      bundle.observation_source =
          std::make_unique<FileObservationSource>(config.observation_source);
      break;
    case ObservationSourceKind::Onnx:
      bundle.observation_source =
          std::make_unique<OnnxObservationSource>(config.observation_source);
      break;
    case ObservationSourceKind::None:
    default:
      bundle.observation_source = std::make_unique<NullObservationSource>();
      break;
  }

  bundle.object_pipeline = std::make_unique<OaObjectPipelineAdapter>();

  if (config.visualizer.enabled) {
    bundle.visualizer =
        std::make_unique<PangolinVisualizer>(quit_check);
  }

  if (config.agent_gateway.enabled) {
    bundle.agent_gateway = std::make_unique<NullAgentGateway>();
  }

  return bundle;
}

}  // namespace oaslam
