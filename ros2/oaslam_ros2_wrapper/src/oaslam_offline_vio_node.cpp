#include <Eigen/Geometry>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <atomic>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "oaslam/core/frame_packet.h"
#include "session_config_utils.h"

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
namespace {

/// Maximum number of recent depth images to keep in the sliding window.
constexpr size_t kMaxDepthWindowSize = 10;

/// Maximum number of IMU samples to buffer.
constexpr size_t kMaxImuBufferSize = 20000;

// ─────────────────────────────────────────────────────────────────────────────
// Deserialization helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Deserialize a serialized ROS2 message into a concrete type.
template <typename MsgT>
std::shared_ptr<MsgT> DeserializeMessage(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_msg) {
  rclcpp::Serialization<MsgT> serializer;
  auto msg = std::make_shared<MsgT>();
  rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
  serializer.deserialize_message(&serialized_msg, msg.get());
  return msg;
}

/// Convert ROS2 header stamp to seconds.
double StampToSeconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

/// Convert nanosecond bag timestamp to seconds.
double NanosToSeconds(int64_t nanos) {
  return static_cast<double>(nanos) * 1e-9;
}

/// A timestamped serialized depth message kept in the sliding window.
struct TimestampedDepth {
  double timestamp_sec;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg;
};

std::string TopicSummary(const std::map<std::string, std::string>& topic_type_map,
                         const std::string& topic_name) {
  const auto it = topic_type_map.find(topic_name);
  if (it == topic_type_map.end()) {
    return "missing:" + topic_name;
  }
  return "ok:" + topic_name + " [" + it->second + "]";
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Main offline VIO node
// ─────────────────────────────────────────────────────────────────────────────

class OaSlamOfflineVioNode : public rclcpp::Node {
 public:
  explicit OaSlamOfflineVioNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("oaslam_offline_vio_node", options) {
    topics_ = oaslam_ros2_wrapper::DeclareOfflineTopicParameters(*this);
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
    semantic_map_markers_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            topics_.publisher.semantic_map_markers_topic,
            rclcpp::QoS(1).reliable().transient_local());

    oaslam_ros2_wrapper::LogNodeStartup(
        get_logger(), "offline VIO", runtime_.session_params,
        {{"Bag path", topics_.bag_path},
         {"RGB topic", topics_.shared.rgb_topic},
         {"Depth topic", topics_.shared.depth_topic},
         {"IMU topic", topics_.shared.imu_topic},
         {"Pose topic", topics_.publisher.pose_topic},
         {"Map points topic", topics_.publisher.map_points_topic},
         {"New map points topic", topics_.publisher.new_map_points_topic},
         {"Visible map points topic", topics_.publisher.visible_map_points_topic},
         {"Semantic snapshot topic", topics_.publisher.semantic_map_snapshot_topic},
         {"Semantic delta topic", topics_.publisher.semantic_map_delta_topic},
         {"Semantic markers topic", topics_.publisher.semantic_map_markers_topic},
         {"World frame", topics_.publisher.world_frame_id},
         {"Agent ID", topics_.publisher.agent_id},
         {"Session ID", topics_.publisher.session_id},
         {"Camera ID", topics_.shared.camera_id},
         {"Output folder", runtime_.session_params.output_folder}});
  }

  ~OaSlamOfflineVioNode() override {
    oaslam_ros2_wrapper::ShutdownNodeRuntime(runtime_, get_logger());
  }

  /// Run the full offline pipeline: open bag -> stream -> process -> save.
  void Run() {
    RCLCPP_INFO(get_logger(), "Bag input | path=%s", topics_.bag_path.c_str());

    // ── Open the bag ──
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = topics_.bag_path;

    rosbag2_cpp::ConverterOptions converter_opts;
    converter_opts.input_serialization_format = "cdr";
    converter_opts.output_serialization_format = "cdr";

    reader.open(storage_opts, converter_opts);

    // ── Collect topic type info and log all topics found in the bag ──
    const auto topics_and_types = reader.get_all_topics_and_types();
    std::map<std::string, std::string> topic_type_map;
    for (const auto& info : topics_and_types) {
      topic_type_map[info.name] = info.type;
    }

    RCLCPP_INFO(get_logger(),
                "Bag scan | topics=%zu | rgb=%s | depth=%s | imu=%s",
                topics_and_types.size(),
                TopicSummary(topic_type_map, topics_.shared.rgb_topic).c_str(),
                TopicSummary(topic_type_map, topics_.shared.depth_topic).c_str(),
                TopicSummary(topic_type_map, topics_.shared.imu_topic).c_str());

    // Verify required topics exist
    if (topic_type_map.find(topics_.shared.rgb_topic) == topic_type_map.end()) {
      RCLCPP_ERROR(get_logger(), "Missing required bag topic | rgb=%s",
                   topics_.shared.rgb_topic.c_str());
      oaslam_ros2_wrapper::ShutdownNodeRuntime(runtime_, get_logger());
      return;
    }
    if (topic_type_map.find(topics_.shared.depth_topic) == topic_type_map.end()) {
      RCLCPP_ERROR(get_logger(), "Missing required bag topic | depth=%s",
                   topics_.shared.depth_topic.c_str());
      oaslam_ros2_wrapper::ShutdownNodeRuntime(runtime_, get_logger());
      return;
    }
    if (topic_type_map.find(topics_.shared.imu_topic) == topic_type_map.end()) {
      RCLCPP_WARN(get_logger(), "IMU topic missing in bag | imu=%s | continuing without IMU",
                  topics_.shared.imu_topic.c_str());
    }

    // ── Streaming state ──
    std::deque<oaslam::ImuMeasurement> imu_buffer;
    std::deque<TimestampedDepth> depth_window;

    // Counters
    uint64_t frame_counter = 0;
    uint64_t total_rgb_in_bag = 0;
    uint64_t total_frames_processed = 0;
    uint64_t total_frames_skipped_no_depth = 0;
    uint64_t total_poses_obtained = 0;
    uint64_t total_poses_interpolated = 0;
    uint64_t total_imu_in_bag = 0;
    uint64_t total_depth_in_bag = 0;
    uint64_t imu_nan_filtered = 0;
    uint64_t total_ignored_messages = 0;

    // Last known good pose
    oaslam::Transform4d last_known_pose = oaslam::Transform4d::eye();

    bool quit_requested = false;
    auto wall_start = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "Bag playback started");

    // ── Single-pass streaming loop ──
    // The rosbag2 sequential reader delivers messages in storage order
    // (typically chronological across all topics).  Messages arrive
    // interleaved: IMU, IMU, RGB, Depth, IMU, ...
    //
    // For each message:
    //   - IMU:   deserialize, filter NaN, push to imu_buffer
    //   - Depth: push serialized message to depth_window (evict oldest)
    //   - RGB:   TRIGGER PROCESSING — find closest depth, drain IMU,
    //            build FramePacket, process, write trajectory line

    while (rclcpp::ok() && reader.has_next() && !quit_requested) {
      auto bag_msg = reader.read_next();
      const std::string& topic = bag_msg->topic_name;

      // ────────────────────────────────────────────────────────────────
      // IMU message
      // ────────────────────────────────────────────────────────────────
      if (topic == topics_.shared.imu_topic) {
        total_imu_in_bag++;

        auto imu_msg = DeserializeMessage<sensor_msgs::msg::Imu>(bag_msg);
        oaslam::ImuMeasurement m;
        m.timestamp = StampToSeconds(imu_msg->header.stamp);
        m.acc_x = imu_msg->linear_acceleration.x;
        m.acc_y = imu_msg->linear_acceleration.y;
        m.acc_z = imu_msg->linear_acceleration.z;
        m.gyro_x = imu_msg->angular_velocity.x;
        m.gyro_y = imu_msg->angular_velocity.y;
        m.gyro_z = imu_msg->angular_velocity.z;

        // Filter NaN/Inf
        if (!std::isfinite(m.acc_x) || !std::isfinite(m.acc_y) ||
            !std::isfinite(m.acc_z) || !std::isfinite(m.gyro_x) ||
            !std::isfinite(m.gyro_y) || !std::isfinite(m.gyro_z) ||
            !std::isfinite(m.timestamp)) {
          imu_nan_filtered++;
          continue;
        }

        imu_buffer.push_back(m);

        while (imu_buffer.size() > kMaxImuBufferSize) {
          imu_buffer.pop_front();
        }

        continue;
      }

      // ────────────────────────────────────────────────────────────────
      // Depth message
      // ────────────────────────────────────────────────────────────────
      if (topic == topics_.shared.depth_topic) {
        total_depth_in_bag++;

        const double ts = NanosToSeconds(bag_msg->time_stamp);
        depth_window.push_back({ts, bag_msg});

        while (depth_window.size() > kMaxDepthWindowSize) {
          depth_window.pop_front();
        }

        continue;
      }

      // ────────────────────────────────────────────────────────────────
      // RGB message — triggers frame processing
      // ────────────────────────────────────────────────────────────────
      if (topic == topics_.shared.rgb_topic) {
        total_rgb_in_bag++;

        const double image_timestamp = NanosToSeconds(bag_msg->time_stamp);

        // ── 1. Find closest depth image in the sliding window ──
        size_t best_depth_idx = 0;
        bool found_depth = false;

        for (size_t d = 0; d < depth_window.size(); ++d) {
          const double dt = std::abs(depth_window[d].timestamp_sec - image_timestamp);
          if (!found_depth ||
              dt < std::abs(depth_window[best_depth_idx].timestamp_sec - image_timestamp)) {
            best_depth_idx = d;
            found_depth = true;
          }
        }

        // ── 2. Null depth guard: skip frame if no depth is buffered ──
        if (!found_depth) {
          total_frames_skipped_no_depth++;
          if (total_frames_skipped_no_depth <= 10) {
            RCLCPP_WARN(get_logger(),
                        "Frame skipped | id=%lu t=%.6f | reason=no-depth-candidate | depth_window=%zu",
                        static_cast<unsigned long>(frame_counter),
                        image_timestamp,
                        depth_window.size());
          }
          if (total_frames_skipped_no_depth == 10) {
            RCLCPP_WARN(get_logger(), "Further no-depth skip warnings suppressed");
          }

          // Write fallback trajectory line for skipped frame
          total_poses_interpolated++;
          if (runtime_.tum_trajectory_file.is_open()) {
            oaslam_ros2_wrapper::WriteTumPoseLine(
                runtime_.tum_trajectory_file, image_timestamp, last_known_pose);
          }
          frame_counter++;
          continue;  // SKIP — do NOT pass null depth to SLAM
        }

        // ── 3. Deserialize RGB ──
        auto rgb_msg = DeserializeMessage<sensor_msgs::msg::Image>(bag_msg);
        cv_bridge::CvImageConstPtr cv_rgb;
        try {
          cv_rgb = cv_bridge::toCvCopy(rgb_msg, "bgr8");
        } catch (const cv_bridge::Exception& exc) {
          RCLCPP_WARN(get_logger(), "Frame skipped | id=%lu | rgb-convert err=%s",
                      static_cast<unsigned long>(frame_counter), exc.what());
          total_poses_interpolated++;
          if (runtime_.tum_trajectory_file.is_open()) {
            oaslam_ros2_wrapper::WriteTumPoseLine(
                runtime_.tum_trajectory_file, image_timestamp, last_known_pose);
          }
          frame_counter++;
          continue;
        }

        // ── 4. Deserialize depth ──
        cv::Mat depth_mat;
        bool has_depth = false;
        {
          auto depth_msg = DeserializeMessage<sensor_msgs::msg::Image>(
              depth_window[best_depth_idx].msg);
          try {
            auto cv_depth = cv_bridge::toCvCopy(depth_msg);
            depth_mat = cv_depth->image;
            has_depth = true;
          } catch (const cv_bridge::Exception& exc) {
            RCLCPP_WARN(get_logger(), "Frame skipped | id=%lu | depth-convert err=%s",
                        static_cast<unsigned long>(frame_counter), exc.what());
          }
        }

        // ── 5. Validate depth dimensions match RGB ──
        if (has_depth) {
          if (depth_mat.empty()) {
            RCLCPP_WARN(get_logger(),
                        "Frame skipped | id=%lu | reason=empty-depth",
                        static_cast<unsigned long>(frame_counter));
            has_depth = false;
          } else if (depth_mat.rows != cv_rgb->image.rows ||
                     depth_mat.cols != cv_rgb->image.cols) {
            RCLCPP_WARN(get_logger(),
                        "Frame skipped | id=%lu | reason=depth-size-mismatch | depth=%dx%d rgb=%dx%d",
                        static_cast<unsigned long>(frame_counter),
                        depth_mat.cols, depth_mat.rows,
                        cv_rgb->image.cols, cv_rgb->image.rows);
            has_depth = false;
          }
        }

        // ── 6. Final null-depth guard: skip if no valid depth ──
        if (!has_depth) {
          total_frames_skipped_no_depth++;
          total_poses_interpolated++;
          if (runtime_.tum_trajectory_file.is_open()) {
            oaslam_ros2_wrapper::WriteTumPoseLine(
                runtime_.tum_trajectory_file, image_timestamp, last_known_pose);
          }
          frame_counter++;
          continue;  // SKIP — do NOT pass invalid depth to SLAM
        }

        // ── 7. Evict stale depth entries ──
        // Depth entries older than the matched one will never be a better
        // match for any future RGB (which has a later timestamp).
        while (depth_window.size() > 1 &&
               depth_window.front().timestamp_sec <
                   depth_window[best_depth_idx].timestamp_sec) {
          if (best_depth_idx > 0) {
            depth_window.pop_front();
            best_depth_idx--;
          } else {
            break;
          }
        }

        // ── 8. Drain IMU up to image timestamp (pre-integration batch) ──
        // Collect all IMU measurements between the previous frame and the
        // current frame.  These are passed as a vector to the SLAM system
        // which performs IMU pre-integration internally.
        //
        // IMPORTANT: We must keep the last (boundary) IMU measurement in the
        // buffer so the next frame can use it as the starting point for its
        // preintegration window.  ORB-SLAM3's PreintegrateIMU() expects at
        // least one IMU sample with timestamp >= prevFrame.mTimeStamp in the
        // queue.  Draining it away causes "Empty IMU measurements vector!!!"
        // warnings and cascading IMU initialization failures.
        std::vector<oaslam::ImuMeasurement> imu_for_frame;
        while (imu_buffer.size() > 1 &&
               imu_buffer.front().timestamp < image_timestamp) {
          imu_for_frame.push_back(imu_buffer.front());
          imu_buffer.pop_front();
        }
        // Include the boundary measurement (at or just past image timestamp)
        // but keep it in the buffer for the next frame's preintegration.
        if (!imu_buffer.empty()) {
          imu_for_frame.push_back(imu_buffer.front());
          // Do NOT pop_front — next frame needs this as its starting point
        }

        // ── 9. Build FramePacket ──
        oaslam::FramePacket frame;
        frame.frame_id = frame_counter++;
        frame.timestamp = image_timestamp;
        frame.camera_id = topics_.shared.camera_id;
        frame.image = cv_rgb->image.clone();
        frame.depth_image = depth_mat.clone();
        frame.has_depth = true;
        frame.imu_measurements = std::move(imu_for_frame);
        frame.has_imu = !frame.imu_measurements.empty();

        // ── 10. Backpressure ──
        if (!oaslam_ros2_wrapper::WaitForMappingBackpressure(*runtime_.session)) {
          RCLCPP_INFO(get_logger(), "Bag playback interrupted | reason=shutdown");
          quit_requested = true;
          continue;
        }

        // ── 11. Process frame ──
        const auto result = runtime_.session->processFrame(frame);
        total_frames_processed++;

        if (!shutdown_requested_->load() && rclcpp::ok()) {
          PublishSemanticMap(rgb_msg->header, result.tracking);
        }

        // ── 12. Write trajectory line (ALWAYS — 1:1 with RGB images) ──
        if (result.tracking.has_pose) {
          total_poses_obtained++;
          last_known_pose = result.tracking.T_world_camera;
          if (!shutdown_requested_->load() && rclcpp::ok()) {
            pose_publisher_->publish(oaslam_ros2_wrapper::ToPoseStamped(
                rgb_msg->header,
                topics_.publisher.world_frame_id,
                result.tracking.T_world_camera));
            map_points_publisher_->publish(oaslam_ros2_wrapper::ToPointCloud2(
                rgb_msg->header, topics_.publisher.world_frame_id,
                result.tracking.scene.map_points));
            new_map_points_publisher_->publish(
                oaslam_ros2_wrapper::ToPointCloud2(
                    rgb_msg->header, topics_.publisher.world_frame_id,
                    result.tracking.scene.new_map_points));
            visible_map_points_publisher_->publish(
                oaslam_ros2_wrapper::ToPointCloud2(
                    rgb_msg->header, topics_.publisher.world_frame_id,
                    result.tracking.scene.visible_map_points));
          }
        } else {
          total_poses_interpolated++;
        }

        if (runtime_.tum_trajectory_file.is_open()) {
          oaslam_ros2_wrapper::WriteTumPoseLine(
              runtime_.tum_trajectory_file, image_timestamp, last_known_pose);
        }

        // ── 13. Progress logging ──
        if (total_frames_processed % 100 == 0) {
          auto now = std::chrono::steady_clock::now();
          double elapsed_sec = std::chrono::duration<double>(now - wall_start).count();
          double fps = total_frames_processed / elapsed_sec;
          std::printf("Progress | frames=%lu tracked=%lu fallback=%lu skip_no_depth=%lu fps=%.1f\n",
                      static_cast<unsigned long>(total_frames_processed),
                      static_cast<unsigned long>(total_poses_obtained),
                      static_cast<unsigned long>(total_poses_interpolated),
                      static_cast<unsigned long>(total_frames_skipped_no_depth),
                      fps);
        }

        // Check if SLAM requested quit
        if (result.quit_requested) {
          RCLCPP_INFO(get_logger(), "SLAM requested quit");
          quit_requested = true;
        }

        continue;
      }

      // Other topics — count but don't process
      total_ignored_messages++;
    }

    // ── Done ──
    auto wall_end = std::chrono::steady_clock::now();
    double total_sec = std::chrono::duration<double>(wall_end - wall_start).count();

    if (!rclcpp::ok() && !quit_requested) {
      RCLCPP_INFO(get_logger(), "Bag playback stopped | reason=shutdown");
    }

    std::printf(
        "Offline VIO done | rgb=%lu depth=%lu imu=%lu imu_nan=%lu "
        "processed=%lu tracked=%lu fallback=%lu skip_no_depth=%lu traj=%lu "
        "total_time=%.1fs fps=%.1f\n",
        static_cast<unsigned long>(total_rgb_in_bag),
        static_cast<unsigned long>(total_depth_in_bag),
        static_cast<unsigned long>(total_imu_in_bag),
        static_cast<unsigned long>(imu_nan_filtered),
        static_cast<unsigned long>(total_frames_processed),
        static_cast<unsigned long>(total_poses_obtained),
        static_cast<unsigned long>(total_poses_interpolated),
        static_cast<unsigned long>(total_frames_skipped_no_depth),
        static_cast<unsigned long>(total_rgb_in_bag),
        total_sec,
        total_frames_processed > 0 ? total_frames_processed / total_sec : 0.0
    );
    oaslam_ros2_wrapper::ShutdownNodeRuntime(runtime_, get_logger());
  }

 private:
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
    semantic_map_markers_publisher_->publish(
        oaslam_ros2_wrapper::ToSemanticMapMarkers(
            header, topics_.publisher, tracking.semantic_map));
  }

  oaslam_ros2_wrapper::NodeRuntime runtime_;
  oaslam_ros2_wrapper::OfflineTopicParams topics_;
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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      semantic_map_markers_publisher_;
  std::shared_ptr<std::atomic<bool>> shutdown_requested_ =
      std::make_shared<std::atomic<bool>>(false);
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<OaSlamOfflineVioNode>(rclcpp::NodeOptions{});

    // Run the offline pipeline (blocking — processes the entire bag)
    node->Run();

  } catch (const std::exception& exc) {
    RCLCPP_FATAL(rclcpp::get_logger("oaslam_offline_vio_node"),
                 "Startup failed | err=%s", exc.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
