#include "src/adapters/orbslam3/orbslam3_pose_utils.h"

#include "src/adapters/orbslam3/internal/include/ImuTypes.h"

namespace oaslam {
namespace orbslam3_utils {

std::vector<ORB_SLAM3::IMU::Point> ToImuPoints(
    const std::vector<ImuMeasurement>& measurements) {
  std::vector<ORB_SLAM3::IMU::Point> result;
  result.reserve(measurements.size());
  for (const auto& m : measurements) {
    // IMU::Point(acc_x, acc_y, acc_z, ang_vel_x, ang_vel_y, ang_vel_z, timestamp)
    result.emplace_back(
        static_cast<float>(m.acc_x), static_cast<float>(m.acc_y),
        static_cast<float>(m.acc_z), static_cast<float>(m.gyro_x),
        static_cast<float>(m.gyro_y), static_cast<float>(m.gyro_z),
        m.timestamp);
  }
  return result;
}

}  // namespace orbslam3_utils
}  // namespace oaslam
