#ifndef OASLAM_ROS2_WRAPPER_SESSION_CONFIG_UTILS_H
#define OASLAM_ROS2_WRAPPER_SESSION_CONFIG_UTILS_H

#include <fstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "oaslam/core/geometry_types.h"
#include "oaslam/core/session_types.h"

namespace oaslam_ros2_wrapper {

struct CommonSessionParams {
  std::string output_folder;
  std::string vocabulary_file;
  std::string camera_settings_file;
  std::string observation_mode;
  std::string detection_model_path;
  std::string detection_file_path;
  std::string ignored_categories_file;
  std::string relocalization_mode;
  bool use_viewer = false;
  bool use_imu = true;
};

bool IsEmptyPath(const std::string& value);

CommonSessionParams DeclareCommonSessionParameters(rclcpp::Node& node);

oaslam::SessionConfig BuildSessionConfig(const CommonSessionParams& params);

std::string OpenTumTrajectoryFile(std::ofstream& output,
                                  const std::string& output_folder);

void WriteTumPoseLine(std::ofstream& output,
                      double timestamp,
                      const oaslam::Transform4d& transform);

}  // namespace oaslam_ros2_wrapper

#endif  // OASLAM_ROS2_WRAPPER_SESSION_CONFIG_UTILS_H
