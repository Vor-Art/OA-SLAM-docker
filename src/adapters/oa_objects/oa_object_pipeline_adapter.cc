#include "src/adapters/oa_objects/oa_object_pipeline_adapter.h"

#include <filesystem>
#include <fstream>

namespace oaslam {

ObjectPreTrack OaObjectPipelineAdapter::preTrack(const FramePacket& frame,
                                                 const std::vector<Detection2D>& detections) {
  (void)frame;
  (void)detections;
  return {};
}

ObjectStateView OaObjectPipelineAdapter::postTrack(const FramePacket& frame,
                                                   const std::vector<Detection2D>& detections,
                                                   const TrackingResult& tracking) {
  (void)frame;
  (void)tracking;

  ObjectStateView view;
  view.detections = detections;
  view.object_projections.reserve(detections.size());
  for (const auto& detection : detections) {
    const cv::Point2f center(static_cast<float>(detection.bbox.x + detection.bbox.width * 0.5),
                             static_cast<float>(detection.bbox.y + detection.bbox.height * 0.5));
    const cv::Size2f size(static_cast<float>(detection.bbox.width),
                          static_cast<float>(detection.bbox.height));
    view.object_projections.emplace_back(center, size, 0.0F);
  }
  return view;
}

void OaObjectPipelineAdapter::setMode(SessionMode mode) {
  mode_ = mode;
}

void OaObjectPipelineAdapter::reset() {}

void OaObjectPipelineAdapter::loadState(const std::filesystem::path& root) {
  (void)root;
}

void OaObjectPipelineAdapter::saveState(const std::filesystem::path& root) const {
  std::filesystem::create_directories(root);
  std::ofstream output(root / "object_pipeline.yaml");
  output << "mode: " << (mode_ == SessionMode::Localization ? "localization" : "mapping") << "\n";
  output << "implementation: passthrough_v1\n";
}

}  // namespace oaslam
