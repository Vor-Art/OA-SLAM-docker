#ifndef OASLAM_COMMON_EIGEN_UTILS_H
#define OASLAM_COMMON_EIGEN_UTILS_H

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace oaslam {

Eigen::Matrix4d CvTransformToEigen(const cv::Matx44d& transform);
cv::Matx44d EigenTransformToCv(const Eigen::Matrix4d& transform);

}  // namespace oaslam

#endif  // OASLAM_COMMON_EIGEN_UTILS_H
