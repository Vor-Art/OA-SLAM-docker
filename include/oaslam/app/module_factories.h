#ifndef OASLAM_APP_MODULE_FACTORIES_H
#define OASLAM_APP_MODULE_FACTORIES_H

#include <memory>

#include "oaslam/core/session_types.h"
#include "oaslam/ports/agent_gateway.h"
#include "oaslam/ports/object_pipeline.h"
#include "oaslam/ports/observation_source.h"
#include "oaslam/ports/slam_backend.h"
#include "oaslam/ports/visualizer.h"

namespace oaslam {

struct ModuleBundle {
  std::unique_ptr<ISlamBackend> slam_backend;
  std::unique_ptr<IObservationSource> observation_source;
  std::unique_ptr<IObjectPipeline> object_pipeline;
  std::unique_ptr<IVisualizer> visualizer;
  std::unique_ptr<IAgentGateway> agent_gateway;
};

ModuleBundle CreateDefaultModules(const SessionConfig& config);

}  // namespace oaslam

#endif  // OASLAM_APP_MODULE_FACTORIES_H
