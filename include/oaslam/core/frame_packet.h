#ifndef OASLAM_CORE_FRAME_PACKET_H
#define OASLAM_CORE_FRAME_PACKET_H

#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

namespace oaslam {

struct FramePacket {
  uint64_t frame_id = 0;
  double timestamp = 0.0;
  std::string camera_id = "mono0";
  cv::Mat image;
  cv::Mat right_image;
  cv::Mat depth_image;
  bool has_right = false;
  bool has_depth = false;
};

}  // namespace oaslam

#endif  // OASLAM_CORE_FRAME_PACKET_H
