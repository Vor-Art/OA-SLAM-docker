#ifndef OASLAM_ROS2_WRAPPER_SESSION_CONFIG_UTILS_H
#define OASLAM_ROS2_WRAPPER_SESSION_CONFIG_UTILS_H

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "oaslam/app/slam_session.h"
#include "oaslam/core/geometry_types.h"
#include "oaslam/core/session_types.h"

namespace oaslam_ros2_wrapper {

struct PublisherParams {
  std::string pose_topic;
  std::string world_frame_id;
};

struct SharedTopicParams {
  std::string rgb_topic;
  std::string depth_topic;
  std::string imu_topic;
  std::string camera_id;
};

struct OnlineTopicParams {
  SharedTopicParams shared;
  PublisherParams publisher;
};

struct OfflineTopicParams {
  SharedTopicParams shared;
  PublisherParams publisher;
  std::string bag_path;
};

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

struct NodeRuntime {
  CommonSessionParams session_params;
  std::unique_ptr<oaslam::SlamSession> session;
  std::ofstream tum_trajectory_file;
};

bool IsEmptyPath(const std::string& value);

SharedTopicParams DeclareSharedTopicParameters(rclcpp::Node& node);

OnlineTopicParams DeclareOnlineTopicParameters(rclcpp::Node& node);

OfflineTopicParams DeclareOfflineTopicParameters(rclcpp::Node& node);

CommonSessionParams DeclareCommonSessionParameters(rclcpp::Node& node);

NodeRuntime CreateNodeRuntime(rclcpp::Node& node);

oaslam::SessionConfig BuildSessionConfig(const CommonSessionParams& params);

std::string OpenTumTrajectoryFile(std::ofstream& output,
                                  const std::string& output_folder);

void ShutdownNodeRuntime(NodeRuntime& runtime, const rclcpp::Logger& logger);

bool WaitForMappingBackpressure(oaslam::SlamSession& session);

void LogNodeStartup(const rclcpp::Logger& logger,
                    const std::string& node_label,
                    const CommonSessionParams& session_params,
                    const std::vector<std::pair<std::string, std::string>>& entries);

geometry_msgs::msg::PoseStamped ToPoseStamped(
    const std_msgs::msg::Header& header,
    const std::string& world_frame_id,
    const oaslam::Transform4d& transform);

void WriteTumPoseLine(std::ofstream& output,
                      double timestamp,
                      const oaslam::Transform4d& transform);

}  // namespace oaslam_ros2_wrapper

#endif  // OASLAM_ROS2_WRAPPER_SESSION_CONFIG_UTILS_H
