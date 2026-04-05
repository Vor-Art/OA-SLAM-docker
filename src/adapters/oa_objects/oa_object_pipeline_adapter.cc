#include "src/adapters/oa_objects/oa_object_pipeline_adapter.h"

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

void OaObjectPipelineAdapter::reset() {}

}  // namespace oaslam
