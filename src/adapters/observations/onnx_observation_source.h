#ifndef OASLAM_ADAPTERS_OBSERVATIONS_ONNX_OBSERVATION_SOURCE_H
#define OASLAM_ADAPTERS_OBSERVATIONS_ONNX_OBSERVATION_SOURCE_H

#include <memory>

#include "oaslam/core/session_types.h"
#include "oaslam/ports/observation_source.h"

namespace ORB_SLAM2 {
class ImageDetectionsManager;
}

namespace oaslam {

class OnnxObservationSource : public IObservationSource {
 public:
  explicit OnnxObservationSource(const ObservationSourceConfig& config);
  ~OnnxObservationSource() override;

  std::vector<Detection2D> detect(const FramePacket& frame) override;
  void reset() override;

 private:
  std::shared_ptr<ORB_SLAM2::ImageDetectionsManager> detector_;
};

}  // namespace oaslam

#endif  // OASLAM_ADAPTERS_OBSERVATIONS_ONNX_OBSERVATION_SOURCE_H
