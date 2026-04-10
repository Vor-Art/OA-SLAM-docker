#include "src/adapters/observations/onnx_observation_source.h"

#include "src/adapters/orbslam3/internal/include/ImageDetections.h"
#include "src/common/opencv_utils.h"

namespace oaslam {

OnnxObservationSource::OnnxObservationSource(const ObservationSourceConfig& config) {
  detector_ = std::make_shared<ORB_SLAM3::ObjectDetector>(
      config.source_path, config.ignored_categories, config.model_input_width,
      config.model_input_height);
}

OnnxObservationSource::~OnnxObservationSource() = default;

std::vector<Detection2D> OnnxObservationSource::detect(const FramePacket& frame) {
  std::vector<Detection2D> detections;
  if (!detector_) {
    return detections;
  }

  const auto legacy = detector_->detect(frame.image);
  detections.reserve(legacy.size());
  for (const auto& item : legacy) {
    detections.push_back(Detection2D{item->category_id, static_cast<float>(item->score),
                                     EigenBBoxToRect(item->bbox)});
  }
  return detections;
}

void OnnxObservationSource::reset() {}

}  // namespace oaslam
