#include "src/adapters/visualization/pangolin_visualizer.h"

#include <utility>

namespace oaslam {

PangolinVisualizer::PangolinVisualizer(std::function<bool()> should_quit_callback)
    : should_quit_callback_(std::move(should_quit_callback)) {}

void PangolinVisualizer::publish(const VisualizationFrame& frame) {
  (void)frame;
}

bool PangolinVisualizer::shouldQuit() const {
  if (!should_quit_callback_) {
    return false;
  }
  return should_quit_callback_();
}

void PangolinVisualizer::shutdown() {
  is_shutdown_ = true;
}

}  // namespace oaslam
