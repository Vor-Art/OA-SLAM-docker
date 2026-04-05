#include "src/adapters/observations/file_observation_source.h"

#include "src/adapters/orbslam2/internal/include/ImageDetections.h"
#include "src/common/opencv_utils.h"

namespace oaslam {

FileObservationSource::FileObservationSource(const ObservationSourceConfig& config) {
  detector_ = std::make_shared<ORB_SLAM2::DetectionsFromFile>(config.source_path,
                                                              config.ignored_categories);
}

FileObservationSource::~FileObservationSource() = default;

std::vector<Detection2D> FileObservationSource::detect(const FramePacket& frame) {
  std::vector<Detection2D> detections;
  if (!detector_) {
    return detections;
  }

  const auto legacy = detector_->detect(static_cast<unsigned int>(frame.frame_id));
  detections.reserve(legacy.size());
  for (const auto& item : legacy) {
    detections.push_back(Detection2D{item->category_id, static_cast<float>(item->score),
                                     EigenBBoxToRect(item->bbox)});
  }
  return detections;
}

void FileObservationSource::reset() {}

}  // namespace oaslam
