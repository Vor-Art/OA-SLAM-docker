#ifndef OASLAM_CORE_FRAME_PACKET_H
#define OASLAM_CORE_FRAME_PACKET_H

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace oaslam {

struct ImuMeasurement {
  double timestamp = 0.0;
  double acc_x = 0.0, acc_y = 0.0, acc_z = 0.0;   // m/s^2
  double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0; // rad/s
};

struct FramePacket {
  uint64_t frame_id = 0;
  double timestamp = 0.0;
  std::string camera_id = "mono0";
  cv::Mat image;
  cv::Mat right_image;
  cv::Mat depth_image;
  bool has_right = false;
  bool has_depth = false;
  std::vector<ImuMeasurement> imu_measurements;
  bool has_imu = false;
};

}  // namespace oaslam

#endif  // OASLAM_CORE_FRAME_PACKET_H
