#ifndef OASLAM_ROS2_WRAPPER_SESSION_CONFIG_UTILS_H
#define OASLAM_ROS2_WRAPPER_SESSION_CONFIG_UTILS_H

#include <array>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <oaslam_ros2_wrapper/msg/semantic_map_delta.hpp>
#include <oaslam_ros2_wrapper/msg/semantic_map_snapshot.hpp>
#include <oaslam_ros2_wrapper/msg/semantic_object.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shared_semantic_map_interfaces/msg/local_semantic_map_delta.hpp>
#include <shared_semantic_map_interfaces/msg/local_semantic_map_snapshot.hpp>
#include <shared_semantic_map_interfaces/msg/local_semantic_object.hpp>
#include <shared_semantic_map_interfaces/msg/local_pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "oaslam/app/slam_session.h"
#include "oaslam/core/geometry_types.h"
#include "oaslam/core/session_types.h"

namespace oaslam_ros2_wrapper {

struct PublisherParams {
  std::string pose_topic;
  std::string map_points_topic;
  std::string new_map_points_topic;
  std::string visible_map_points_topic;
  std::string semantic_map_snapshot_topic;
  std::string semantic_map_delta_topic;
  std::string semantic_map_markers_topic;
  std::string local_semantic_map_snapshot_topic;
  std::string local_semantic_map_delta_topic;
  std::string local_pose_topic;
  std::string world_frame_id;
  std::string agent_id;
  std::string session_id;
};

struct DepthAlignmentParams {
  bool enabled = false;
  bool rotation_is_column_major = false;
  double depth_unit_scale = 0.001;
  std::array<double, 4> rgb_intrinsics{0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> depth_intrinsics{0.0, 0.0, 0.0, 0.0};
  std::array<double, 9> depth_to_rgb_rotation{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};
  std::array<double, 3> depth_to_rgb_translation{0.0, 0.0, 0.0};
};

struct SharedTopicParams {
  std::string rgb_topic;
  std::string depth_topic;
  std::string imu_topic;
  std::string camera_id;
  DepthAlignmentParams depth_alignment;
};

struct OnlineTopicParams {
  SharedTopicParams shared;
  PublisherParams publisher;
};

struct OfflineTopicParams {
  SharedTopicParams shared;
  PublisherParams publisher;
  std::string bag_path;
  double start_offset_sec = 0.0;
  double finish_time_sec = -1.0;
};

struct CommonSessionParams {
  std::string output_folder;
  std::string vocabulary_file;
  std::string camera_settings_file;
  std::string observation_mode;
  std::string detection_model_path;
  std::string detection_file_path;
  std::string ignored_categories_file;
  bool ignore_dynamic_categories = true;
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

geometry_msgs::msg::Pose ToPose(const oaslam::Transform4d& transform);

shared_semantic_map_interfaces::msg::LocalPose ToLocalPoseMsg(
    const std_msgs::msg::Header& header,
    const PublisherParams& params,
    std::uint64_t map_id,
    std::uint64_t sequence,
    const oaslam::Transform4d& transform);

sensor_msgs::msg::PointCloud2 ToPointCloud2(
    const std_msgs::msg::Header& header,
    const std::string& world_frame_id,
    const std::vector<cv::Point3d>& points);

msg::SemanticObject ToSemanticObjectMsg(
    const oaslam::SemanticObject& object);

msg::SemanticMapSnapshot ToSemanticMapSnapshotMsg(
    const std_msgs::msg::Header& header,
    const PublisherParams& params,
    const oaslam::SemanticMapSnapshot& snapshot);

msg::SemanticMapDelta ToSemanticMapDeltaMsg(
    const std_msgs::msg::Header& header,
    const PublisherParams& params,
    const oaslam::SemanticMapDelta& delta);

shared_semantic_map_interfaces::msg::LocalSemanticObject ToLocalSemanticObjectMsg(
    const oaslam::SemanticObject& object);

shared_semantic_map_interfaces::msg::LocalSemanticMapSnapshot
ToLocalSemanticMapSnapshotMsg(const std_msgs::msg::Header& header,
                              const PublisherParams& params,
                              const oaslam::SemanticMapSnapshot& snapshot);

shared_semantic_map_interfaces::msg::LocalSemanticMapDelta
ToLocalSemanticMapDeltaMsg(const std_msgs::msg::Header& header,
                           const PublisherParams& params,
                           const oaslam::SemanticMapDelta& delta);

visualization_msgs::msg::MarkerArray ToSemanticMapMarkers(
    const std_msgs::msg::Header& header,
    const PublisherParams& params,
    const oaslam::SemanticMapSnapshot& snapshot);

void WriteTumPoseLine(std::ofstream& output,
                      double timestamp,
                      const oaslam::Transform4d& transform);

}  // namespace oaslam_ros2_wrapper

#endif  // OASLAM_ROS2_WRAPPER_SESSION_CONFIG_UTILS_H
