#ifndef OASLAM_ROS2_WRAPPER_DEPTH_ALIGNMENT_H
#define OASLAM_ROS2_WRAPPER_DEPTH_ALIGNMENT_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include <opencv2/core.hpp>

#include "session_config_utils.h"

namespace oaslam_ros2_wrapper {

inline bool ReadDepthValue(const cv::Mat& depth,
                           int row,
                           int col,
                           double* raw_depth) {
  if (depth.type() == CV_16UC1) {
    const auto value = depth.at<std::uint16_t>(row, col);
    if (value == 0) {
      return false;
    }
    *raw_depth = static_cast<double>(value);
    return true;
  }

  if (depth.type() == CV_32FC1) {
    const float value = depth.at<float>(row, col);
    if (!std::isfinite(value) || value <= 0.0F) {
      return false;
    }
    *raw_depth = static_cast<double>(value);
    return true;
  }

  return false;
}

inline bool WriteAlignedDepthValue(cv::Mat& aligned_depth,
                                   cv::Mat& z_buffer,
                                   int row,
                                   int col,
                                   double z_rgb_m,
                                   double depth_unit_scale) {
  if (z_rgb_m <= 0.0 || !std::isfinite(z_rgb_m)) {
    return false;
  }

  float& current_z = z_buffer.at<float>(row, col);
  if (z_rgb_m >= static_cast<double>(current_z)) {
    return false;
  }

  const double raw_depth = z_rgb_m / depth_unit_scale;
  if (!std::isfinite(raw_depth) || raw_depth <= 0.0) {
    return false;
  }

  if (aligned_depth.type() == CV_16UC1) {
    const double max_depth =
        static_cast<double>(std::numeric_limits<std::uint16_t>::max());
    if (raw_depth > max_depth) {
      return false;
    }

    aligned_depth.at<std::uint16_t>(row, col) =
        static_cast<std::uint16_t>(std::lround(raw_depth));
  } else if (aligned_depth.type() == CV_32FC1) {
    aligned_depth.at<float>(row, col) = static_cast<float>(raw_depth);
  } else {
    return false;
  }

  current_z = static_cast<float>(z_rgb_m);
  return true;
}

inline bool AlignDepthToRgb(const cv::Mat& depth,
                            const cv::Size& rgb_size,
                            const DepthAlignmentParams& params,
                            cv::Mat* aligned_depth,
                            std::string* error) {
  if (depth.type() != CV_16UC1 && depth.type() != CV_32FC1) {
    if (error != nullptr) {
      *error = "depth alignment supports only 16UC1 and 32FC1 depth images";
    }
    return false;
  }

  if (rgb_size.width <= 0 || rgb_size.height <= 0) {
    if (error != nullptr) {
      *error = "RGB image size is invalid";
    }
    return false;
  }

  cv::Mat result(rgb_size, depth.type(), cv::Scalar(0));
  cv::Mat z_buffer(
      rgb_size, CV_32FC1,
      cv::Scalar(std::numeric_limits<float>::infinity()));

  const auto& kd = params.depth_intrinsics;
  const auto& kr = params.rgb_intrinsics;
  const auto& r = params.depth_to_rgb_rotation;
  const auto& t = params.depth_to_rgb_translation;
  const auto r_at = [&r, &params](int row, int col) {
    return params.rotation_is_column_major ? r[col * 3 + row]
                                           : r[row * 3 + col];
  };
  for (int v_depth = 0; v_depth < depth.rows; ++v_depth) {
    for (int u_depth = 0; u_depth < depth.cols; ++u_depth) {
      double raw_depth = 0.0;
      if (!ReadDepthValue(depth, v_depth, u_depth, &raw_depth)) {
        continue;
      }

      const double z_depth_m = raw_depth * params.depth_unit_scale;
      const double x_depth_m =
          (static_cast<double>(u_depth) - kd[2]) * z_depth_m / kd[0];
      const double y_depth_m =
          (static_cast<double>(v_depth) - kd[3]) * z_depth_m / kd[1];

      const double x_rgb_m =
          r_at(0, 0) * x_depth_m + r_at(0, 1) * y_depth_m +
          r_at(0, 2) * z_depth_m + t[0];
      const double y_rgb_m =
          r_at(1, 0) * x_depth_m + r_at(1, 1) * y_depth_m +
          r_at(1, 2) * z_depth_m + t[1];
      const double z_rgb_m =
          r_at(2, 0) * x_depth_m + r_at(2, 1) * y_depth_m +
          r_at(2, 2) * z_depth_m + t[2];
      if (z_rgb_m <= 0.0) {
        continue;
      }

      const int u_rgb =
          static_cast<int>(std::lround(kr[0] * x_rgb_m / z_rgb_m + kr[2]));
      const int v_rgb =
          static_cast<int>(std::lround(kr[1] * y_rgb_m / z_rgb_m + kr[3]));
      if (u_rgb < 0 || u_rgb >= rgb_size.width ||
          v_rgb < 0 || v_rgb >= rgb_size.height) {
        continue;
      }

      WriteAlignedDepthValue(result, z_buffer, v_rgb, u_rgb, z_rgb_m,
                             params.depth_unit_scale);
    }
  }

  *aligned_depth = std::move(result);
  return true;
}

inline bool PrepareDepthForRgb(const cv::Mat& depth,
                               const cv::Size& rgb_size,
                               const DepthAlignmentParams& params,
                               cv::Mat* prepared_depth,
                               std::string* error) {
  if (depth.empty()) {
    if (error != nullptr) {
      *error = "empty-depth";
    }
    return false;
  }

  if (!params.enabled) {
    if (depth.size() != rgb_size) {
      if (error != nullptr) {
        *error = "depth-size-mismatch";
      }
      return false;
    }
    *prepared_depth = depth.clone();
    return true;
  }

  return AlignDepthToRgb(depth, rgb_size, params, prepared_depth, error);
}

}  // namespace oaslam_ros2_wrapper

#endif  // OASLAM_ROS2_WRAPPER_DEPTH_ALIGNMENT_H
