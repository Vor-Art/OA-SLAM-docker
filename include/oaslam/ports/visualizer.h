#ifndef OASLAM_PORTS_VISUALIZER_H
#define OASLAM_PORTS_VISUALIZER_H

#include "oaslam/core/visualization_types.h"

namespace oaslam {

class IVisualizer {
 public:
  virtual ~IVisualizer() = default;
  virtual void publish(const VisualizationFrame& frame) = 0;
  virtual bool shouldQuit() const = 0;
  virtual void shutdown() = 0;
};

}  // namespace oaslam

#endif  // OASLAM_PORTS_VISUALIZER_H
