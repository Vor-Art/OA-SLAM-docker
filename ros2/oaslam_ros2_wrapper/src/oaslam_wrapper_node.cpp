#include <Eigen/Geometry>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <chrono>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <std_msgs/msg/header.hpp>

#include "oaslam/app/module_factories.h"
#include "oaslam/app/slam_session.h"

namespace {

bool IsEmptyPath(const std::string& value) {
  return value.empty() || value == "none" || value == "null";
}

void EnsureFileExists(const std::string& path, const std::string& label) {
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error(label + " does not exist: " + path);
  }
}

std::vector<int> LoadIgnoredCategories(const std::string& path) {
  std::vector<int> categories;
  if (IsEmptyPath(path)) {
    return categories;
  }

  EnsureFileExists(path, "Ignored categories file");
  std::ifstream input(path);
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }

    std::istringstream stream(line);
    int category = 0;
    if (stream >> category) {
      categories.push_back(category);
    }
  }
  return categories;
}

oaslam::ObservationSourceKind ParseObservationMode(const std::string& value) {
  if (value == "none") {
    return oaslam::ObservationSourceKind::None;
  }
  if (value == "onnx") {
    return oaslam::ObservationSourceKind::Onnx;
  }
  throw std::runtime_error("observation_mode must be 'none' or 'onnx'");
}

oaslam::RelocalizationMode ParseRelocalizationMode(const std::string& value) {
  if (value == "objects") {
    return oaslam::RelocalizationMode::Objects;
  }
  if (value == "points+objects") {
    return oaslam::RelocalizationMode::PointsAndObjects;
  }
  if (value == "points") {
    return oaslam::RelocalizationMode::Points;
  }
  throw std::runtime_error(
      "relocalization_mode must be 'points', 'objects', or 'points+objects'");
}

geometry_msgs::msg::PoseStamped ToPoseStamped(const std_msgs::msg::Header& header,
                                              const std::string& world_frame_id,
                                              const oaslam::Transform4d& transform) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  pose.header.frame_id = world_frame_id;
  pose.pose.position.x = transform(0, 3);
  pose.pose.position.y = transform(1, 3);
  pose.pose.position.z = transform(2, 3);

  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation(row, col) = transform(row, col);
    }
  }

  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  pose.pose.orientation.x = quaternion.x();
  pose.pose.orientation.y = quaternion.y();
  pose.pose.orientation.z = quaternion.z();
  pose.pose.orientation.w = quaternion.w();
  return pose;
}

/// Write a single pose line in TUM format: timestamp tx ty tz qx qy qz qw
void WriteTumPoseLine(std::ofstream& out, double timestamp,
                      const oaslam::Transform4d& transform) {
  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation(row, col) = transform(row, col);
    }
  }

  Eigen::Quaterniond q(rotation);
  q.normalize();

  out << std::fixed << std::setprecision(6) << timestamp << " "
      << std::setprecision(9)
      << transform(0, 3) << " " << transform(1, 3) << " " << transform(2, 3) << " "
      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
}

}  // namespace

class OaSlamWrapperNode : public rclcpp::Node {
 public:
  explicit OaSlamWrapperNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("oaslam_wrapper", options) {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/oa_slam/camera_pose");
    world_frame_id_ = declare_parameter<std::string>("world_frame_id", "map");
    camera_id_ = declare_parameter<std::string>("camera_id", "mono0");
    const std::string output_folder =
        declare_parameter<std::string>("output_folder", "");

    const std::string vocabulary_file =
        declare_parameter<std::string>("vocabulary_file", "/opt/OA-SLAM/Vocabulary/ORBvoc.txt");
    const std::string camera_config_file = declare_parameter<std::string>(
        "camera_config_file", "/opt/OA-SLAM/ros2/oaslam_ros2_wrapper/config/slam_camera.yaml");
    const std::string observation_mode =
        declare_parameter<std::string>("observation_mode", "none");
    const std::string onnx_model_path =
        declare_parameter<std::string>("onnx_model_path", "");
    const std::string ignored_categories_file =
        declare_parameter<std::string>("ignored_categories_file", "");
    const std::string relocalization_mode =
        declare_parameter<std::string>("relocalization_mode", "points+objects");
    const bool use_viewer = declare_parameter<bool>("use_viewer", false);

    EnsureFileExists(vocabulary_file, "Vocabulary file");
    EnsureFileExists(camera_config_file, "Camera config file");

    oaslam::SessionConfig session_config;
    session_config.slam_backend.vocabulary_file = vocabulary_file;
    session_config.slam_backend.camera_settings_file = camera_config_file;
    session_config.slam_backend.use_viewer = use_viewer;
    session_config.slam_backend.use_ar_viewer = false;
    session_config.slam_backend.use_objects_in_local_ba = 0;
    session_config.slam_backend.relocalization_mode =
        ParseRelocalizationMode(relocalization_mode);
    session_config.visualizer.enabled = use_viewer;
    session_config.agent_gateway.enabled = false;

    session_config.observation_source.kind = ParseObservationMode(observation_mode);
    session_config.observation_source.ignored_categories =
        LoadIgnoredCategories(ignored_categories_file);
    const oaslam::ModelInputSize model_input_size = oaslam::LoadModelInputSize(camera_config_file);
    session_config.observation_source.model_input_width = model_input_size.width;
    session_config.observation_source.model_input_height = model_input_size.height;
    if (session_config.observation_source.kind == oaslam::ObservationSourceKind::Onnx) {
      if (IsEmptyPath(onnx_model_path)) {
        throw std::runtime_error("onnx_model_path is required when observation_mode is 'onnx'");
      }
      EnsureFileExists(onnx_model_path, "ONNX model");
      session_config.observation_source.source_path = onnx_model_path;
    }

    session_ = std::make_unique<oaslam::SlamSession>(
        session_config, oaslam::CreateDefaultModules(session_config));

    // Open TUM trajectory file if output folder is specified
    if (!IsEmptyPath(output_folder)) {
      std::filesystem::create_directories(output_folder);
      const std::string tum_path = output_folder + "/CameraTrajectory.txt";
      tum_trajectory_file_.open(tum_path, std::ios::out | std::ios::trunc);
      if (!tum_trajectory_file_.is_open()) {
        throw std::runtime_error("Failed to open TUM trajectory file: " + tum_path);
      }
      tum_trajectory_file_ << "# TUM trajectory format: timestamp tx ty tz qx qy qz qw\n";
      RCLCPP_INFO(get_logger(), "Saving camera trajectory (TUM format) to: %s", tum_path.c_str());
    }

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    image_subscription_ = image_transport::create_subscription(
        this, image_topic_,
        std::bind(&OaSlamWrapperNode::HandleImage, this, std::placeholders::_1), "raw",
        rmw_qos_profile_sensor_data);
    missing_image_timer_ = create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&OaSlamWrapperNode::WarnIfNoImagesReceived, this));

    RCLCPP_INFO(get_logger(), "OA-SLAM ROS2 wrapper subscribed to %s and publishing poses on %s",
                image_topic_.c_str(), pose_topic_.c_str());
  }

  ~OaSlamWrapperNode() override {
    if (tum_trajectory_file_.is_open()) {
      tum_trajectory_file_.flush();
      tum_trajectory_file_.close();
      RCLCPP_INFO(get_logger(), "Camera trajectory file closed.");
    }
    if (session_) {
      session_->shutdown();
    }
  }

 private:
  void HandleImage(const sensor_msgs::msg::Image::ConstSharedPtr& message) {
    received_first_image_ = true;

    cv_bridge::CvImageConstPtr cv_image;
    try {
      cv_image = cv_bridge::toCvShare(message);
    } catch (const cv_bridge::Exception& exc) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Failed to convert image message: %s", exc.what());
      return;
    }

    oaslam::FramePacket frame;
    frame.frame_id = frame_counter_++;
    frame.timestamp = rclcpp::Time(message->header.stamp).seconds();
    frame.camera_id = camera_id_;
    frame.image = cv_image->image.clone();

    const auto result = session_->processFrame(frame);
    if (!result.tracking.has_pose) {
      return;
    }

    pose_publisher_->publish(
        ToPoseStamped(message->header, world_frame_id_, result.tracking.T_world_camera));

    // Write pose to TUM trajectory file
    if (tum_trajectory_file_.is_open()) {
      WriteTumPoseLine(tum_trajectory_file_, frame.timestamp, result.tracking.T_world_camera);
    }
  }

  void WarnIfNoImagesReceived() {
    if (received_first_image_) {
      missing_image_timer_->cancel();
      return;
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "No images received on %s yet", image_topic_.c_str());
  }

  std::unique_ptr<oaslam::SlamSession> session_;
  image_transport::Subscriber image_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr missing_image_timer_;
  std::ofstream tum_trajectory_file_;
  std::string image_topic_;
  std::string pose_topic_;
  std::string world_frame_id_;
  std::string camera_id_;
  std::uint64_t frame_counter_ = 0;
  bool received_first_image_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<OaSlamWrapperNode>(options);
    rclcpp::spin(node);
  } catch (const std::exception& exc) {
    RCLCPP_FATAL(rclcpp::get_logger("oaslam_wrapper"), "Failed to start wrapper: %s",
                 exc.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
