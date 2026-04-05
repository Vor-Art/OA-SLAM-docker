#include "src/common/eigen_utils.h"

namespace oaslam {

Eigen::Matrix4d CvTransformToEigen(const cv::Matx44d& transform) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result(i, j) = transform(i, j);
    }
  }
  return result;
}

cv::Matx44d EigenTransformToCv(const Eigen::Matrix4d& transform) {
  cv::Matx44d result = cv::Matx44d::eye();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result(i, j) = transform(i, j);
    }
  }
  return result;
}

}  // namespace oaslam
