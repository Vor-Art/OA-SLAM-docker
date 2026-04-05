#ifndef OASLAM_COMMON_OPENCV_UTILS_H
#define OASLAM_COMMON_OPENCV_UTILS_H

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "oaslam/core/geometry_types.h"

namespace ORB_SLAM2 {
using BBox2 = Eigen::Vector4d;
}

namespace oaslam {

Transform4d CvMatToTransform(const cv::Mat& matrix);
cv::Mat TransformToCvMat(const Transform4d& transform);
cv::Rect2d EigenBBoxToRect(const ORB_SLAM2::BBox2& bbox);
ORB_SLAM2::BBox2 RectToEigenBBox(const cv::Rect2d& rect);
double EstimateMeanDepthCameraZ(const cv::Mat& Tcw, const std::vector<cv::Point3d>& world_points);

}  // namespace oaslam

#endif  // OASLAM_COMMON_OPENCV_UTILS_H
