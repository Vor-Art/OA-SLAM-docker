#ifndef OASLAM_PORTS_AGENT_GATEWAY_H
#define OASLAM_PORTS_AGENT_GATEWAY_H

namespace oaslam {

class IAgentGateway {
 public:
  virtual ~IAgentGateway() = default;
  virtual void publishLocalDelta() = 0;
  virtual void pollRemoteDeltas() = 0;
  virtual void shutdown() = 0;
};

}  // namespace oaslam

#endif  // OASLAM_PORTS_AGENT_GATEWAY_H
