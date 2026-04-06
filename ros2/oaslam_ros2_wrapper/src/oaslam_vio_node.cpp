#include <Eigen/Geometry>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cstdint>
#include <chrono>
#include <filesystem>
#include <functional>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <std_msgs/msg/header.hpp>

#include "imu_buffer.h"
#include "oaslam/app/module_factories.h"
#include "oaslam/app/slam_session.h"
#include "oaslam/core/frame_packet.h"

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
  if (value == "file") {
    return oaslam::ObservationSourceKind::File;
  }
  throw std::runtime_error("observation_mode must be 'none', 'onnx', or 'file'");
}

oaslam::RelocalizationMode ParseRelocalizationMode(const std::string& value) {
  if (value == "objects") {
    return oaslam::RelocalizationMode::Objects;
  }
  if (value == "points_and_objects" || value == "points+objects") {
    return oaslam::RelocalizationMode::PointsAndObjects;
  }
  if (value == "points") {
    return oaslam::RelocalizationMode::Points;
  }
  throw std::runtime_error(
      "relocalization_mode must be 'points', 'objects', or 'points_and_objects'");
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

}  // namespace

class OaSlamVioNode : public rclcpp::Node {
 public:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image>;

  explicit OaSlamVioNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("oaslam_vio_node", options) {
    // Declare ROS2 parameters
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    depth_topic_ =
        declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/camera/imu");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/oaslam/pose");
    world_frame_id_ = declare_parameter<std::string>("world_frame_id", "map");
    camera_id_ = declare_parameter<std::string>("camera_id", "rgbd0");

    const std::string vocabulary_file =
        declare_parameter<std::string>("vocabulary_file", "/app/Vocabulary/ORBvoc.txt");
    const std::string camera_settings_file =
        declare_parameter<std::string>("camera_settings_file", "/app/config/d435i_imu_rgbd.yaml");
    const std::string observation_mode =
        declare_parameter<std::string>("observation_mode", "none");
    const std::string detection_model_path =
        declare_parameter<std::string>("detection_model_path", "");
    const std::string detection_file_path =
        declare_parameter<std::string>("detection_file_path", "");
    const std::string ignored_categories_file =
        declare_parameter<std::string>("ignored_categories_file", "");
    const std::string relocalization_mode =
        declare_parameter<std::string>("relocalization_mode", "points");
    const bool use_viewer = declare_parameter<bool>("use_viewer", false);
    const bool use_imu = declare_parameter<bool>("use_imu", true);

    EnsureFileExists(vocabulary_file, "Vocabulary file");
    EnsureFileExists(camera_settings_file, "Camera settings file");

    // Configure session for ORB-SLAM3 with IMU
    oaslam::SessionConfig session_config;
    session_config.slam_backend.vocabulary_file = vocabulary_file;
    session_config.slam_backend.camera_settings_file = camera_settings_file;
    session_config.slam_backend.use_viewer = use_viewer;
    session_config.slam_backend.use_ar_viewer = false;
    session_config.slam_backend.use_objects_in_local_ba = 0;
    session_config.slam_backend.relocalization_mode =
        ParseRelocalizationMode(relocalization_mode);
    session_config.slam_backend.kind = oaslam::SlamBackendKind::OrbSlam3;
    session_config.slam_backend.use_imu = use_imu;
    session_config.visualizer.enabled = use_viewer;
    session_config.agent_gateway.enabled = false;

    session_config.observation_source.kind = ParseObservationMode(observation_mode);
    session_config.observation_source.ignored_categories =
        LoadIgnoredCategories(ignored_categories_file);

    if (session_config.observation_source.kind == oaslam::ObservationSourceKind::Onnx) {
      if (IsEmptyPath(detection_model_path)) {
        throw std::runtime_error(
            "detection_model_path is required when observation_mode is 'onnx'");
      }
      EnsureFileExists(detection_model_path, "ONNX detection model");
      session_config.observation_source.source_path = detection_model_path;
    } else if (session_config.observation_source.kind == oaslam::ObservationSourceKind::File) {
      if (IsEmptyPath(detection_file_path)) {
        throw std::runtime_error(
            "detection_file_path is required when observation_mode is 'file'");
      }
      EnsureFileExists(detection_file_path, "Detection file");
      session_config.observation_source.source_path = detection_file_path;
    }

    session_ = std::make_unique<oaslam::SlamSession>(
        session_config, oaslam::CreateDefaultModules(session_config));

    // Set up publishers
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);

    // Set up IMU subscription (independent, not synchronized — arrives at 200Hz)
    rclcpp::QoS imu_qos(100);
    imu_qos.best_effort();
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, imu_qos,
        std::bind(&OaSlamVioNode::HandleImu, this, std::placeholders::_1));

    // Set up synchronized RGB + Depth subscriptions using message_filters
    rclcpp::QoS image_qos(10);
    image_qos.best_effort();

    rgb_sub_.subscribe(this, rgb_topic_, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(
        std::bind(&OaSlamVioNode::HandleSyncedImages, this, std::placeholders::_1,
                  std::placeholders::_2));

    // Timer to warn if no images are received
    missing_image_timer_ = create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&OaSlamVioNode::WarnIfNoImagesReceived, this));

    RCLCPP_INFO(get_logger(),
                "OA-SLAM VIO node started (ORB-SLAM3, IMU=%s)\n"
                "  RGB topic:   %s\n"
                "  Depth topic: %s\n"
                "  IMU topic:   %s\n"
                "  Pose topic:  %s",
                use_imu ? "enabled" : "disabled",
                rgb_topic_.c_str(), depth_topic_.c_str(),
                imu_topic_.c_str(), pose_topic_.c_str());
  }

  ~OaSlamVioNode() override {
    if (session_) {
      session_->shutdown();
    }
  }

 private:
  /// IMU callback — pushes measurements into the thread-safe buffer.
  /// Called at ~200Hz, independently of image callbacks.
  void HandleImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    oaslam::ImuMeasurement m;
    m.timestamp = rclcpp::Time(msg->header.stamp).seconds();
    m.acc_x = msg->linear_acceleration.x;
    m.acc_y = msg->linear_acceleration.y;
    m.acc_z = msg->linear_acceleration.z;
    m.gyro_x = msg->angular_velocity.x;
    m.gyro_y = msg->angular_velocity.y;
    m.gyro_z = msg->angular_velocity.z;
    imu_buffer_.push(m);
  }

  /// Synchronized RGB+Depth callback — called at ~30Hz when both images arrive.
  void HandleSyncedImages(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                          const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
    received_first_image_ = true;

    // Convert RGB image
    cv_bridge::CvImageConstPtr cv_rgb;
    try {
      cv_rgb = cv_bridge::toCvShare(rgb_msg, "bgr8");
    } catch (const cv_bridge::Exception& exc) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Failed to convert RGB image: %s", exc.what());
      return;
    }

    // Convert depth image
    cv_bridge::CvImageConstPtr cv_depth;
    try {
      cv_depth = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception& exc) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Failed to convert depth image: %s", exc.what());
      return;
    }

    const double image_timestamp = rclcpp::Time(rgb_msg->header.stamp).seconds();

    // Drain IMU measurements up to the image timestamp
    auto imu_measurements = imu_buffer_.drainUntil(image_timestamp);

    // Build FramePacket with RGB, depth, and IMU data
    oaslam::FramePacket frame;
    frame.frame_id = frame_counter_++;
    frame.timestamp = image_timestamp;
    frame.camera_id = camera_id_;
    frame.image = cv_rgb->image.clone();
    frame.depth_image = cv_depth->image.clone();
    frame.has_depth = true;
    frame.imu_measurements = std::move(imu_measurements);
    frame.has_imu = !frame.imu_measurements.empty();

    // Process frame through the OA-SLAM pipeline
    const auto result = session_->processFrame(frame);
    if (!result.tracking.has_pose) {
      return;
    }

    // Publish pose
    pose_publisher_->publish(
        ToPoseStamped(rgb_msg->header, world_frame_id_, result.tracking.T_world_camera));
  }

  void WarnIfNoImagesReceived() {
    if (received_first_image_) {
      missing_image_timer_->cancel();
      return;
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "No synchronized RGB+Depth images received yet.\n"
                         "  RGB topic:   %s\n"
                         "  Depth topic: %s\n"
                         "  IMU topic:   %s (buffered: %zu)",
                         rgb_topic_.c_str(), depth_topic_.c_str(),
                         imu_topic_.c_str(), imu_buffer_.size());
  }

  // SLAM session
  std::unique_ptr<oaslam::SlamSession> session_;

  // IMU buffer (thread-safe, shared between IMU callback and image callback)
  oaslam_ros2::ImuBuffer imu_buffer_;

  // message_filters subscribers and synchronizer
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // IMU subscription (independent, not synchronized)
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

  // Pose publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

  // Timer for missing image warning
  rclcpp::TimerBase::SharedPtr missing_image_timer_;

  // Topic names
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string imu_topic_;
  std::string pose_topic_;
  std::string world_frame_id_;
  std::string camera_id_;

  // State
  std::uint64_t frame_counter_ = 0;
  bool received_first_image_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<OaSlamVioNode>(options);
    rclcpp::spin(node);
  } catch (const std::exception& exc) {
    RCLCPP_FATAL(rclcpp::get_logger("oaslam_vio_node"), "Failed to start VIO node: %s",
                 exc.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
