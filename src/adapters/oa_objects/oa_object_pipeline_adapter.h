#ifndef OASLAM_ADAPTERS_OA_OBJECTS_OA_OBJECT_PIPELINE_ADAPTER_H
#define OASLAM_ADAPTERS_OA_OBJECTS_OA_OBJECT_PIPELINE_ADAPTER_H

#include "oaslam/ports/object_pipeline.h"

namespace oaslam {

class OaObjectPipelineAdapter : public IObjectPipeline {
 public:
  OaObjectPipelineAdapter() = default;
  ~OaObjectPipelineAdapter() override = default;

  ObjectPreTrack preTrack(const FramePacket& frame,
                          const std::vector<Detection2D>& detections) override;
  ObjectStateView postTrack(const FramePacket& frame,
                            const std::vector<Detection2D>& detections,
                            const TrackingResult& tracking) override;
  void reset() override;
};

}  // namespace oaslam

#endif  // OASLAM_ADAPTERS_OA_OBJECTS_OA_OBJECT_PIPELINE_ADAPTER_H
