#include "src/common/opencv_utils.h"

#include <Eigen/Dense>

namespace oaslam {

Transform4d CvMatToTransform(const cv::Mat& matrix) {
  Transform4d transform = Transform4d::eye();
  if (matrix.rows < 4 || matrix.cols < 4) {
    return transform;
  }

  const cv::Mat Tcw = matrix.clone();
  cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
  cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
  cv::Mat Rwc = Rcw.t();
  cv::Mat twc = -Rwc * tcw;

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      transform(i, j) = Rwc.at<float>(i, j);
    }
    transform(i, 3) = twc.at<float>(i);
  }

  return transform;
}

cv::Mat TransformToCvMat(const Transform4d& transform) {
  cv::Mat matrix = cv::Mat::eye(4, 4, CV_32F);
  cv::Matx33d Rwc(transform(0, 0), transform(0, 1), transform(0, 2), transform(1, 0),
                  transform(1, 1), transform(1, 2), transform(2, 0), transform(2, 1),
                  transform(2, 2));
  cv::Vec3d twc(transform(0, 3), transform(1, 3), transform(2, 3));
  cv::Matx33d Rcw = Rwc.t();
  cv::Vec3d tcw = -(Rcw * twc);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      matrix.at<float>(i, j) = static_cast<float>(Rcw(i, j));
    }
    matrix.at<float>(i, 3) = static_cast<float>(tcw(i));
  }
  return matrix;
}

cv::Rect2d EigenBBoxToRect(const ORB_SLAM2::BBox2& bbox) {
  return cv::Rect2d(bbox[0], bbox[1], bbox[2] - bbox[0], bbox[3] - bbox[1]);
}

ORB_SLAM2::BBox2 RectToEigenBBox(const cv::Rect2d& rect) {
  return ORB_SLAM2::BBox2(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
}

double EstimateMeanDepthCameraZ(const cv::Mat& Tcw, const std::vector<cv::Point3d>& world_points) {
  if (Tcw.rows < 4 || Tcw.cols < 4 || world_points.empty()) {
    return 0.0;
  }

  double total_depth = 0.0;
  std::size_t count = 0;
  for (const auto& point : world_points) {
    const double x = Tcw.at<float>(0, 0) * point.x + Tcw.at<float>(0, 1) * point.y +
                     Tcw.at<float>(0, 2) * point.z + Tcw.at<float>(0, 3);
    const double y = Tcw.at<float>(1, 0) * point.x + Tcw.at<float>(1, 1) * point.y +
                     Tcw.at<float>(1, 2) * point.z + Tcw.at<float>(1, 3);
    const double z = Tcw.at<float>(2, 0) * point.x + Tcw.at<float>(2, 1) * point.y +
                     Tcw.at<float>(2, 2) * point.z + Tcw.at<float>(2, 3);
    (void)x;
    (void)y;
    if (z > 0.0) {
      total_depth += z;
      ++count;
    }
  }

  return count == 0 ? 0.0 : total_depth / static_cast<double>(count);
}

}  // namespace oaslam
