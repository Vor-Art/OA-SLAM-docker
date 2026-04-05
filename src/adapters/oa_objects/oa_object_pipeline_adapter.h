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
  void setMode(SessionMode mode) override;
  void reset() override;
  void loadState(const std::filesystem::path& root) override;
  void saveState(const std::filesystem::path& root) const override;

 private:
  SessionMode mode_ = SessionMode::Mapping;
};

}  // namespace oaslam

#endif  // OASLAM_ADAPTERS_OA_OBJECTS_OA_OBJECT_PIPELINE_ADAPTER_H
