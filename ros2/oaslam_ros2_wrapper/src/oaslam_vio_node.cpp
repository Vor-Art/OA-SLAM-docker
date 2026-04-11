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

#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <std_msgs/msg/header.hpp>

#include "imu_buffer.h"
#include "oaslam/app/module_factories.h"
#include "oaslam/app/slam_session.h"
#include "oaslam/core/frame_packet.h"
#include "session_config_utils.h"

namespace {

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

/// Maximum number of keyframes allowed in the LocalMapping queue before
/// the ROS2 callback starts waiting.  Keeping this small ensures that
/// LocalMapping is never far behind Tracking.
constexpr int kMaxKeyframeQueueDepth = 2;

/// How long to sleep (µs) between queue-depth polls while waiting for
/// LocalMapping to drain.
constexpr int kBackpressureSleepUs = 50;

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
    const auto common_params =
        oaslam_ros2_wrapper::DeclareCommonSessionParameters(*this);

    // Open TUM trajectory file if output folder is specified
    const std::string tum_path =
        oaslam_ros2_wrapper::OpenTumTrajectoryFile(
            tum_trajectory_file_, common_params.output_folder);
    if (!tum_path.empty()) {
      RCLCPP_INFO(get_logger(), "Saving camera trajectory (TUM format) to: %s", tum_path.c_str());
    }
    const oaslam::SessionConfig session_config =
        oaslam_ros2_wrapper::BuildSessionConfig(common_params);

    session_ = std::make_unique<oaslam::SlamSession>(
        session_config, oaslam::CreateDefaultModules(session_config));

    // Set up publishers
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);

    // Set up IMU subscription (independent, not synchronized — arrives at 200Hz)
    // Large queue to buffer IMU during backpressure waits
    rclcpp::QoS imu_qos(500);
    imu_qos.best_effort();
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, imu_qos,
        std::bind(&OaSlamVioNode::HandleImu, this, std::placeholders::_1));

    // Set up synchronized RGB + Depth subscriptions using message_filters
    // Large queue (500) to prevent message drops during backpressure waits
    rmw_qos_profile_t sensor_qos_500 = rmw_qos_profile_sensor_data;
    sensor_qos_500.depth = 500;

    rgb_sub_.subscribe(this, rgb_topic_, sensor_qos_500);
    depth_sub_.subscribe(this, depth_topic_, sensor_qos_500);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(500), rgb_sub_, depth_sub_);
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
                "  Pose topic:  %s\n"
                "  output_folder: %s",
                common_params.use_imu ? "enabled" : "disabled",
                rgb_topic_.c_str(), depth_topic_.c_str(),
                imu_topic_.c_str(), pose_topic_.c_str(),
                common_params.output_folder.c_str());
  }

  ~OaSlamVioNode() override {
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

    // Filter out NaN/Inf IMU readings (can come from faulty sensor data)
    if (!std::isfinite(m.acc_x) || !std::isfinite(m.acc_y) || !std::isfinite(m.acc_z) ||
        !std::isfinite(m.gyro_x) || !std::isfinite(m.gyro_y) || !std::isfinite(m.gyro_z) ||
        !std::isfinite(m.timestamp)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Dropping IMU message with NaN/Inf values");
      return;
    }

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

    // Backpressure: wait until LocalMapping has drained its keyframe queue.
    // This ensures that regardless of bag playback speed (-r), the Tracking
    // thread always sees the same LocalMapping state (idle vs busy), making
    // SLAM results deterministic.
    while (session_->keyframesInQueue() > kMaxKeyframeQueueDepth) {
      usleep(kBackpressureSleepUs);
    }

    // Process frame through the OA-SLAM pipeline
    const auto result = session_->processFrame(frame);
    if (!result.tracking.has_pose) {
      return;
    }

    // Publish pose
    pose_publisher_->publish(
        ToPoseStamped(rgb_msg->header, world_frame_id_, result.tracking.T_world_camera));

    if (tum_trajectory_file_.is_open()) {
      oaslam_ros2_wrapper::WriteTumPoseLine(
          tum_trajectory_file_, image_timestamp, result.tracking.T_world_camera);
    }
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

  // TUM trajectory output
  std::ofstream tum_trajectory_file_;

  // State
  std::uint64_t frame_counter_ = 0;
  bool received_first_image_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<OaSlamVioNode>(rclcpp::NodeOptions{});
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
