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
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <cstdint>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "depth_alignment.h"
#include "imu_buffer.h"
#include "oaslam/core/frame_packet.h"
#include "session_config_utils.h"

class OaSlamVioNode : public rclcpp::Node {
 public:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image>;

  explicit OaSlamVioNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("oaslam_vio_node", options) {
    topics_ = oaslam_ros2_wrapper::DeclareOnlineTopicParameters(*this);
    runtime_ = oaslam_ros2_wrapper::CreateNodeRuntime(*this);
    rclcpp::on_shutdown(
        [flag = shutdown_requested_]() { flag->store(true); },
        get_node_base_interface()->get_context());

    // Set up publishers
    pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(topics_.publisher.pose_topic, 10);
    map_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        topics_.publisher.map_points_topic, 10);
    new_map_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        topics_.publisher.new_map_points_topic, 10);
    visible_map_points_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(
            topics_.publisher.visible_map_points_topic, 10);
    semantic_map_snapshot_publisher_ =
        create_publisher<oaslam_ros2_wrapper::msg::SemanticMapSnapshot>(
            topics_.publisher.semantic_map_snapshot_topic,
            rclcpp::QoS(1).reliable().transient_local());
    semantic_map_delta_publisher_ =
        create_publisher<oaslam_ros2_wrapper::msg::SemanticMapDelta>(
            topics_.publisher.semantic_map_delta_topic, 10);
    local_semantic_map_snapshot_publisher_ =
        create_publisher<shared_semantic_map_interfaces::msg::LocalSemanticMapSnapshot>(
            topics_.publisher.local_semantic_map_snapshot_topic,
            rclcpp::QoS(1).reliable().transient_local());
    local_semantic_map_delta_publisher_ =
        create_publisher<shared_semantic_map_interfaces::msg::LocalSemanticMapDelta>(
            topics_.publisher.local_semantic_map_delta_topic, 10);
    semantic_map_markers_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            topics_.publisher.semantic_map_markers_topic,
            rclcpp::QoS(1).reliable().transient_local());

    // Set up IMU subscription (independent, not synchronized — arrives at 200Hz)
    // Large queue to buffer IMU during backpressure waits
    rclcpp::QoS imu_qos(500);
    imu_qos.best_effort();
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        topics_.shared.imu_topic, imu_qos,
        std::bind(&OaSlamVioNode::HandleImu, this, std::placeholders::_1));

    // Set up synchronized RGB + Depth subscriptions using message_filters
    // Large queue (500) to prevent message drops during backpressure waits
    rmw_qos_profile_t sensor_qos_500 = rmw_qos_profile_sensor_data;
    sensor_qos_500.depth = 500;

    rgb_sub_.subscribe(this, topics_.shared.rgb_topic, sensor_qos_500);
    depth_sub_.subscribe(this, topics_.shared.depth_topic, sensor_qos_500);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(500), rgb_sub_, depth_sub_);
    sync_->registerCallback(
        std::bind(&OaSlamVioNode::HandleSyncedImages, this, std::placeholders::_1,
                  std::placeholders::_2));

    // Timer to warn if no images are received
    missing_image_timer_ = create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&OaSlamVioNode::WarnIfNoImagesReceived, this));

    oaslam_ros2_wrapper::LogNodeStartup(
        get_logger(), "online VIO", runtime_.session_params,
        {{"RGB topic", topics_.shared.rgb_topic},
         {"Depth topic", topics_.shared.depth_topic},
         {"IMU topic", topics_.shared.imu_topic},
         {"Pose topic", topics_.publisher.pose_topic},
         {"Map points topic", topics_.publisher.map_points_topic},
         {"New map points topic", topics_.publisher.new_map_points_topic},
         {"Visible map points topic", topics_.publisher.visible_map_points_topic},
         {"Semantic snapshot topic", topics_.publisher.semantic_map_snapshot_topic},
         {"Semantic delta topic", topics_.publisher.semantic_map_delta_topic},
         {"Local semantic snapshot topic",
          topics_.publisher.local_semantic_map_snapshot_topic},
         {"Local semantic delta topic",
          topics_.publisher.local_semantic_map_delta_topic},
         {"Semantic markers topic", topics_.publisher.semantic_map_markers_topic},
         {"World frame", topics_.publisher.world_frame_id},
         {"Agent ID", topics_.publisher.agent_id},
         {"Session ID", topics_.publisher.session_id},
         {"Camera ID", topics_.shared.camera_id},
         {"Depth alignment", topics_.shared.depth_alignment.enabled ? "on" : "off"},
         {"Output folder", runtime_.session_params.output_folder}});
  }

  ~OaSlamVioNode() override {
    oaslam_ros2_wrapper::ShutdownNodeRuntime(runtime_, get_logger());
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
                           "Dropped IMU sample | reason=non-finite");
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
                            "RGB conversion failed | err=%s", exc.what());
      return;
    }

    // Convert depth image
    cv_bridge::CvImageConstPtr cv_depth;
    try {
      cv_depth = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception& exc) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Depth conversion failed | err=%s", exc.what());
      return;
    }

    cv::Mat depth_for_rgb;
    std::string depth_error;
    if (!oaslam_ros2_wrapper::PrepareDepthForRgb(
            cv_depth->image, cv_rgb->image.size(),
            topics_.shared.depth_alignment, &depth_for_rgb, &depth_error)) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Depth preparation failed | reason=%s | depth=%dx%d rgb=%dx%d alignment=%s",
          depth_error.c_str(),
          cv_depth->image.cols, cv_depth->image.rows,
          cv_rgb->image.cols, cv_rgb->image.rows,
          topics_.shared.depth_alignment.enabled ? "on" : "off");
      return;
    }

    const double image_timestamp = rclcpp::Time(rgb_msg->header.stamp).seconds();

    // Drain IMU measurements up to the image timestamp
    auto imu_measurements = imu_buffer_.drainUntil(image_timestamp);

    // Build FramePacket with RGB, depth, and IMU data
    oaslam::FramePacket frame;
    frame.frame_id = frame_counter_++;
    frame.timestamp = image_timestamp;
    frame.camera_id = topics_.shared.camera_id;
    frame.image = cv_rgb->image.clone();
    frame.depth_image = std::move(depth_for_rgb);
    frame.has_depth = true;
    frame.imu_measurements = std::move(imu_measurements);
    frame.has_imu = !frame.imu_measurements.empty();

    // Backpressure: wait until LocalMapping has drained its keyframe queue.
    // This ensures that regardless of bag playback speed (-r), the Tracking
    // thread always sees the same LocalMapping state (idle vs busy), making
    // SLAM results deterministic.
    if (!oaslam_ros2_wrapper::WaitForMappingBackpressure(*runtime_.session)) {
      RCLCPP_INFO(get_logger(), "Stopping frame processing | reason=shutdown");
      return;
    }

    // Process frame through the OA-SLAM pipeline
    const auto result = runtime_.session->processFrame(frame);
    if (shutdown_requested_->load() || !rclcpp::ok()) {
      return;
    }

    PublishSemanticMap(rgb_msg->header, result.tracking);

    if (!result.tracking.has_pose) {
      return;
    }

    // Publish pose
    pose_publisher_->publish(
        oaslam_ros2_wrapper::ToPoseStamped(
            rgb_msg->header,
            topics_.publisher.world_frame_id,
            result.tracking.T_world_camera));
    map_points_publisher_->publish(oaslam_ros2_wrapper::ToPointCloud2(
        rgb_msg->header, topics_.publisher.world_frame_id,
        result.tracking.scene.map_points));
    new_map_points_publisher_->publish(oaslam_ros2_wrapper::ToPointCloud2(
        rgb_msg->header, topics_.publisher.world_frame_id,
        result.tracking.scene.new_map_points));
    visible_map_points_publisher_->publish(oaslam_ros2_wrapper::ToPointCloud2(
        rgb_msg->header, topics_.publisher.world_frame_id,
        result.tracking.scene.visible_map_points));

    if (runtime_.tum_trajectory_file.is_open()) {
      oaslam_ros2_wrapper::WriteTumPoseLine(
          runtime_.tum_trajectory_file, image_timestamp, result.tracking.T_world_camera);
    }
  }

  void PublishSemanticMap(const std_msgs::msg::Header& header,
                          const oaslam::TrackingResult& tracking) {
    if (tracking.semantic_map_delta.empty()) {
      return;
    }

    semantic_map_snapshot_publisher_->publish(
        oaslam_ros2_wrapper::ToSemanticMapSnapshotMsg(
            header, topics_.publisher, tracking.semantic_map));
    semantic_map_delta_publisher_->publish(
        oaslam_ros2_wrapper::ToSemanticMapDeltaMsg(
            header, topics_.publisher, tracking.semantic_map_delta));
    local_semantic_map_snapshot_publisher_->publish(
        oaslam_ros2_wrapper::ToLocalSemanticMapSnapshotMsg(
            header, topics_.publisher, tracking.semantic_map));
    local_semantic_map_delta_publisher_->publish(
        oaslam_ros2_wrapper::ToLocalSemanticMapDeltaMsg(
            header, topics_.publisher, tracking.semantic_map_delta));
    semantic_map_markers_publisher_->publish(
        oaslam_ros2_wrapper::ToSemanticMapMarkers(
            header, topics_.publisher, tracking.semantic_map));
  }

  void WarnIfNoImagesReceived() {
    if (received_first_image_) {
      missing_image_timer_->cancel();
      return;
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Waiting for synchronized input | rgb=%s depth=%s imu=%s imu_buffer=%zu",
                         topics_.shared.rgb_topic.c_str(),
                         topics_.shared.depth_topic.c_str(),
                         topics_.shared.imu_topic.c_str(),
                         imu_buffer_.size());
  }

  oaslam_ros2_wrapper::NodeRuntime runtime_;
  oaslam_ros2_wrapper::OnlineTopicParams topics_;

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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      new_map_points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      visible_map_points_publisher_;
  rclcpp::Publisher<oaslam_ros2_wrapper::msg::SemanticMapSnapshot>::SharedPtr
      semantic_map_snapshot_publisher_;
  rclcpp::Publisher<oaslam_ros2_wrapper::msg::SemanticMapDelta>::SharedPtr
      semantic_map_delta_publisher_;
  rclcpp::Publisher<shared_semantic_map_interfaces::msg::LocalSemanticMapSnapshot>::SharedPtr
      local_semantic_map_snapshot_publisher_;
  rclcpp::Publisher<shared_semantic_map_interfaces::msg::LocalSemanticMapDelta>::SharedPtr
      local_semantic_map_delta_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      semantic_map_markers_publisher_;

  // Timer for missing image warning
  rclcpp::TimerBase::SharedPtr missing_image_timer_;

  // State
  std::uint64_t frame_counter_ = 0;
  bool received_first_image_ = false;
  std::shared_ptr<std::atomic<bool>> shutdown_requested_ =
      std::make_shared<std::atomic<bool>>(false);
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<OaSlamVioNode>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
  } catch (const std::exception& exc) {
    RCLCPP_FATAL(rclcpp::get_logger("oaslam_vio_node"), "Startup failed | err=%s",
                 exc.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
