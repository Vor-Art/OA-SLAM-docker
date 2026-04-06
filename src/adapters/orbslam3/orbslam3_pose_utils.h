#ifndef OASLAM_ADAPTERS_ORBSLAM3_ORBSLAM3_POSE_UTILS_H
#define OASLAM_ADAPTERS_ORBSLAM3_ORBSLAM3_POSE_UTILS_H

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>

#include "oaslam/core/geometry_types.h"  // Transform4d = cv::Matx44d
#include "oaslam/core/frame_packet.h"    // ImuMeasurement

// Forward declare ORB-SLAM3 IMU type
namespace ORB_SLAM3 {
namespace IMU {
class Point;
}  // namespace IMU
}  // namespace ORB_SLAM3

namespace oaslam {
namespace orbslam3_utils {

/// Convert ORB-SLAM3 camera-to-world Sophus::SE3f pose (Tcw) to our Transform4d (Twc).
/// ORB-SLAM3's TrackRGBD returns Tcw. We need Twc = Tcw.inverse().
inline Transform4d SophusTcwToTransform4d(const Sophus::SE3f& Tcw) {
  Sophus::SE3f Twc = Tcw.inverse();
  Eigen::Matrix4f mat = Twc.matrix();
  Transform4d result;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      result(r, c) = static_cast<double>(mat(r, c));
  return result;
}

/// Check if a Sophus SE3 pose is valid (not identity with zero translation, which ORB-SLAM3
/// returns on tracking failure).
inline bool IsValidPose(const Sophus::SE3f& pose) {
  // ORB-SLAM3 returns identity-ish matrix on tracking loss
  return pose.translation().norm() > 1e-7f ||
         (pose.rotationMatrix() - Eigen::Matrix3f::Identity()).norm() > 1e-7f;
}

/// Convert Eigen::Vector3f (ORB-SLAM3 MapPoint position) to cv::Point3d
inline cv::Point3d EigenToPoint3d(const Eigen::Vector3f& v) {
  return cv::Point3d(static_cast<double>(v.x()),
                     static_cast<double>(v.y()),
                     static_cast<double>(v.z()));
}

/// Convert our ImuMeasurement vector to ORB-SLAM3 IMU::Point vector.
/// This is declared here but defined in the .cc file since it needs the full IMU::Point definition.
std::vector<ORB_SLAM3::IMU::Point> ToImuPoints(
    const std::vector<ImuMeasurement>& measurements);

}  // namespace orbslam3_utils
}  // namespace oaslam

#endif  // OASLAM_ADAPTERS_ORBSLAM3_ORBSLAM3_POSE_UTILS_H
