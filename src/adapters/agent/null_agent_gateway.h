#ifndef OASLAM_ADAPTERS_AGENT_NULL_AGENT_GATEWAY_H
#define OASLAM_ADAPTERS_AGENT_NULL_AGENT_GATEWAY_H

#include "oaslam/ports/agent_gateway.h"

namespace oaslam {

class NullAgentGateway : public IAgentGateway {
 public:
  void publishLocalDelta() override;
  void pollRemoteDeltas() override;
  void shutdown() override;
};

}  // namespace oaslam

#endif  // OASLAM_ADAPTERS_AGENT_NULL_AGENT_GATEWAY_H
