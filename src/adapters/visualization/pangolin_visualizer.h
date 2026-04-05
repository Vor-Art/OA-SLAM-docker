#ifndef OASLAM_ADAPTERS_VISUALIZATION_PANGOLIN_VISUALIZER_H
#define OASLAM_ADAPTERS_VISUALIZATION_PANGOLIN_VISUALIZER_H

#include <functional>

#include "oaslam/ports/visualizer.h"

namespace oaslam {

class PangolinVisualizer : public IVisualizer {
 public:
  explicit PangolinVisualizer(std::function<bool()> should_quit_callback = {});
  ~PangolinVisualizer() override = default;

  void publish(const VisualizationFrame& frame) override;
  bool shouldQuit() const override;
  void shutdown() override;

 private:
  std::function<bool()> should_quit_callback_;
  bool is_shutdown_ = false;
};

}  // namespace oaslam

#endif  // OASLAM_ADAPTERS_VISUALIZATION_PANGOLIN_VISUALIZER_H
