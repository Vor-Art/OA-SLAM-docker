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

#include <algorithm>
#include <cmath>
#include <cstdint>
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
    
    // Set up publishers
    pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(topics_.publisher.pose_topic, 10);

        oaslam_ros2_wrapper::LogNodeStartup(
        get_logger(), "offline VIO", runtime_.session_params,
        {{"Bag path", topics_.bag_path},
         {"RGB topic", topics_.shared.rgb_topic},
         {"Depth topic", topics_.shared.depth_topic},
         {"IMU topic", topics_.shared.imu_topic},
         {"Pose topic", topics_.publisher.pose_topic},
         {"World frame", topics_.publisher.world_frame_id},
         {"Camera ID", topics_.shared.camera_id},
         {"Output folder", runtime_.session_params.output_folder}});
  }

  ~OaSlamOfflineVioNode() override {
    oaslam_ros2_wrapper::ShutdownNodeRuntime(runtime_, get_logger());
  }

  /// Run the full offline pipeline: open bag -> stream -> process -> save.
  void Run() {
    RCLCPP_INFO(get_logger(), "Opening rosbag: %s", topics_.bag_path.c_str());

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

    RCLCPP_INFO(get_logger(), "Bag contains %zu topics:", topics_and_types.size());
    for (const auto& info : topics_and_types) {
      RCLCPP_INFO(get_logger(), "  topic: '%s'  type: [%s]  serialization: %s",
                  info.name.c_str(), info.type.c_str(),
                  info.serialization_format.c_str());
    }

    RCLCPP_INFO(get_logger(),
                "Topic resolution summary:\n"
                "  RGB topic:   '%s' -> %s\n"
                "  Depth topic: '%s' -> %s\n"
                "  IMU topic:   '%s' -> %s",
                topics_.shared.rgb_topic.c_str(),
                topic_type_map.count(topics_.shared.rgb_topic) ? "FOUND" : "NOT FOUND",
                topics_.shared.depth_topic.c_str(),
                topic_type_map.count(topics_.shared.depth_topic) ? "FOUND" : "NOT FOUND",
                topics_.shared.imu_topic.c_str(),
                topic_type_map.count(topics_.shared.imu_topic) ? "FOUND" : "NOT FOUND");

    // Verify required topics exist
    if (topic_type_map.find(topics_.shared.rgb_topic) == topic_type_map.end()) {
      RCLCPP_ERROR(get_logger(), "RGB topic '%s' not found in bag! Aborting.",
                   topics_.shared.rgb_topic.c_str());
      oaslam_ros2_wrapper::ShutdownNodeRuntime(runtime_, get_logger());
      return;
    }
    if (topic_type_map.find(topics_.shared.depth_topic) == topic_type_map.end()) {
      RCLCPP_ERROR(get_logger(), "Depth topic '%s' not found in bag! Aborting.",
                   topics_.shared.depth_topic.c_str());
      oaslam_ros2_wrapper::ShutdownNodeRuntime(runtime_, get_logger());
      return;
    }
    if (topic_type_map.find(topics_.shared.imu_topic) == topic_type_map.end()) {
      RCLCPP_WARN(get_logger(), "IMU topic '%s' not found in bag. Proceeding without IMU.",
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

    // Track per-topic message counts for final summary
    std::map<std::string, uint64_t> per_topic_msg_count;

    // Track whether we've logged the first message for each topic
    std::map<std::string, bool> first_msg_logged;

    // Last known good pose
    oaslam::Transform4d last_known_pose = oaslam::Transform4d::eye();

    bool quit_requested = false;
    auto wall_start = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "Streaming bag messages...");

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

    while (reader.has_next() && !quit_requested) {
      auto bag_msg = reader.read_next();
      const std::string& topic = bag_msg->topic_name;

      // Track per-topic message counts
      per_topic_msg_count[topic]++;

      // Log the first message seen on each topic (helps debug topic issues)
      if (first_msg_logged.find(topic) == first_msg_logged.end()) {
        first_msg_logged[topic] = true;
        const double msg_ts = NanosToSeconds(bag_msg->time_stamp);
        const std::string type_str = topic_type_map.count(topic)
            ? topic_type_map.at(topic) : "(unknown type)";
        RCLCPP_INFO(get_logger(),
                    "First message on topic '%s' [%s] at t=%.6f s",
                    topic.c_str(), type_str.c_str(), msg_ts);
      }

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
                        "Frame %lu (t=%.6f): no depth candidates in buffer "
                        "(depth_window size=%zu). Skipping frame.",
                        static_cast<unsigned long>(frame_counter),
                        image_timestamp,
                        depth_window.size());
          }
          if (total_frames_skipped_no_depth == 10) {
            RCLCPP_WARN(get_logger(),
                        "Suppressing further 'no depth' warnings (10 already logged).");
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
          RCLCPP_WARN(get_logger(), "Frame %lu: failed to convert RGB: %s",
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
            RCLCPP_WARN(get_logger(), "Frame %lu: failed to convert depth: %s",
                        static_cast<unsigned long>(frame_counter), exc.what());
          }
        }

        // ── 5. Validate depth dimensions match RGB ──
        if (has_depth) {
          if (depth_mat.empty()) {
            RCLCPP_WARN(get_logger(),
                        "Frame %lu: depth image deserialized but is empty. Skipping frame.",
                        static_cast<unsigned long>(frame_counter));
            has_depth = false;
          } else if (depth_mat.rows != cv_rgb->image.rows ||
                     depth_mat.cols != cv_rgb->image.cols) {
            RCLCPP_WARN(get_logger(),
                        "Frame %lu: depth size (%dx%d) != RGB size (%dx%d). Skipping frame.",
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
        oaslam_ros2_wrapper::WaitForMappingBackpressure(*runtime_.session);

        // ── 11. Process frame ──
        const auto result = runtime_.session->processFrame(frame);
        total_frames_processed++;

        // ── 12. Write trajectory line (ALWAYS — 1:1 with RGB images) ──
        if (result.tracking.has_pose) {
          total_poses_obtained++;
          last_known_pose = result.tracking.T_world_camera;
          pose_publisher_->publish(oaslam_ros2_wrapper::ToPoseStamped(
              rgb_msg->header,
              topics_.publisher.world_frame_id,
              result.tracking.T_world_camera));
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
          RCLCPP_INFO(get_logger(),
                      "Progress: %lu frames processed, %lu poses, %lu skipped (no depth), "
                      "%.1f fps",
                      static_cast<unsigned long>(total_frames_processed),
                      static_cast<unsigned long>(total_poses_obtained),
                      static_cast<unsigned long>(total_frames_skipped_no_depth),
                      fps);
        }

        // Check if SLAM requested quit
        if (result.quit_requested) {
          RCLCPP_INFO(get_logger(), "SLAM session requested quit.");
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

    // Log per-topic message counts
    RCLCPP_INFO(get_logger(), "Per-topic message counts read from bag:");
    for (const auto& [tname, tcount] : per_topic_msg_count) {
      const std::string type_str = topic_type_map.count(tname)
          ? topic_type_map.at(tname) : "(unknown)";
      const char* role = "";
      if (tname == topics_.shared.rgb_topic) role = " [RGB]";
      else if (tname == topics_.shared.depth_topic) role = " [DEPTH]";
      else if (tname == topics_.shared.imu_topic) role = " [IMU]";
      else role = " [ignored]";
      RCLCPP_INFO(get_logger(), "  '%s' [%s]: %lu messages%s",
                  tname.c_str(), type_str.c_str(),
                  static_cast<unsigned long>(tcount), role);
    }

    RCLCPP_INFO(get_logger(),
                "\n========================================\n"
                "  Offline VIO processing complete\n"
                "  RGB images in bag: %lu\n"
                "  Depth images in bag: %lu\n"
                "  IMU samples in bag: %lu (NaN filtered: %lu)\n"
                "  Ignored messages:  %lu\n"
                "  Frames processed:  %lu\n"
                "  Frames skipped (no depth): %lu\n"
                "  Poses (tracked):   %lu\n"
                "  Poses (fallback):  %lu\n"
                "  Depth topic:         %s\n"
                "  Trajectory lines:  %lu (1:1 with RGB)\n"
                "  Wall time:         %.1f s\n"
                "  Average FPS:       %.1f\n"
                "========================================",
                static_cast<unsigned long>(total_rgb_in_bag),
                static_cast<unsigned long>(total_depth_in_bag),
                static_cast<unsigned long>(total_imu_in_bag),
                static_cast<unsigned long>(imu_nan_filtered),
                static_cast<unsigned long>(total_ignored_messages),
                static_cast<unsigned long>(total_frames_processed),
                static_cast<unsigned long>(total_frames_skipped_no_depth),
                static_cast<unsigned long>(total_poses_obtained),
                static_cast<unsigned long>(total_poses_interpolated),
                topics_.shared.depth_topic.c_str(),
                static_cast<unsigned long>(total_rgb_in_bag),
                total_sec,
                total_frames_processed > 0 ? total_frames_processed / total_sec : 0.0);
    oaslam_ros2_wrapper::ShutdownNodeRuntime(runtime_, get_logger());
  }

 private:
  oaslam_ros2_wrapper::NodeRuntime runtime_;
  oaslam_ros2_wrapper::OfflineTopicParams topics_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
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
                 "Fatal error: %s", exc.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
