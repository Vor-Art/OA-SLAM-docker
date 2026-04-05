#include "oaslam/app/slam_session.h"

#include <utility>

namespace oaslam {

SlamSession::SlamSession(SessionConfig config, ModuleBundle modules)
    : config_(std::move(config)), modules_(std::move(modules)) {}

SessionFrameResult SlamSession::processFrame(const FramePacket& frame) {
  if (modules_.agent_gateway) {
    modules_.agent_gateway->pollRemoteDeltas();
  }

  std::vector<Detection2D> detections;
  if (modules_.observation_source) {
    detections = modules_.observation_source->detect(frame);
  }

  ObjectPreTrack object_pre_track;
  if (modules_.object_pipeline) {
    object_pre_track = modules_.object_pipeline->preTrack(frame, detections);
  }

  TrackingResult tracking;
  if (modules_.slam_backend) {
    tracking = modules_.slam_backend->processFrame(frame, detections,
                                                   object_pre_track.pose_proposals);
  }

  ObjectStateView objects;
  if (modules_.object_pipeline) {
    objects = modules_.object_pipeline->postTrack(frame, detections, tracking);
  } else {
    objects.detections = detections;
  }

  if (modules_.visualizer) {
    modules_.visualizer->publish(VisualizationFrame{frame, tracking, objects});
  }

  if (modules_.agent_gateway) {
    modules_.agent_gateway->publishLocalDelta();
  }

  SessionFrameResult result;
  result.tracking = std::move(tracking);
  result.objects = std::move(objects);
  result.quit_requested = modules_.visualizer && modules_.visualizer->shouldQuit();
  return result;
}

void SlamSession::reset() {
  if (modules_.observation_source) {
    modules_.observation_source->reset();
  }
  if (modules_.object_pipeline) {
    modules_.object_pipeline->reset();
  }
  if (modules_.slam_backend) {
    modules_.slam_backend->reset();
  }
}

void SlamSession::shutdown() {
  if (is_shutdown_) {
    return;
  }

  if (modules_.visualizer) {
    modules_.visualizer->shutdown();
  }
  if (modules_.agent_gateway) {
    modules_.agent_gateway->shutdown();
  }
  if (modules_.slam_backend) {
    modules_.slam_backend->shutdown();
  }
  is_shutdown_ = true;
}

}  // namespace oaslam
