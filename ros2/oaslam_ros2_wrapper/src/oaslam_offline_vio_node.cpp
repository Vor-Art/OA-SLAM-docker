/// @file oaslam_offline_vio_node.cpp
/// @brief Offline VIO node that reads a ROS2 bag (sqlite3) directly and
///        processes frames sequentially through the OA-SLAM pipeline.
///
/// Messages are consumed incrementally — streamed and processed on the fly
/// in storage order — without loading the entire bag into memory.  This
/// allows processing of very large bags (30 GB+) with bounded memory usage.
///
/// The reader delivers messages in bag-storage order (typically chronological
/// across all topics).  IMU samples are accumulated in a bounded deque, depth
/// images are kept in a small sliding window, and each RGB image triggers
/// immediate frame processing.
///
/// Usage (launch):
///   ros2 launch oaslam_ros2_wrapper oaslam_offline_vio.launch.py \
///       bag_path:=/path/to/rosbag  output_folder:=/path/to/output
///
/// Usage (direct):
///   ros2 run oaslam_ros2_wrapper oaslam_offline_vio_node \
///       --ros-args -p bag_path:=/path/to/rosbag ...

#include <Eigen/Geometry>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "oaslam/app/module_factories.h"
#include "oaslam/app/slam_session.h"
#include "oaslam/core/frame_packet.h"

// ─────────────────────────────────────────────────────────────────────────────
// Helpers (same as in oaslam_vio_node.cpp)
// ─────────────────────────────────────────────────────────────────────────────
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
  if (IsEmptyPath(path)) return categories;
  EnsureFileExists(path, "Ignored categories file");
  std::ifstream input(path);
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::istringstream stream(line);
    int category = 0;
    if (stream >> category) categories.push_back(category);
  }
  return categories;
}

oaslam::ObservationSourceKind ParseObservationMode(const std::string& value) {
  if (value == "none") return oaslam::ObservationSourceKind::None;
  if (value == "onnx") return oaslam::ObservationSourceKind::Onnx;
  if (value == "file") return oaslam::ObservationSourceKind::File;
  throw std::runtime_error("observation_mode must be 'none', 'onnx', or 'file'");
}

oaslam::RelocalizationMode ParseRelocalizationMode(const std::string& value) {
  if (value == "objects") return oaslam::RelocalizationMode::Objects;
  if (value == "points_and_objects" || value == "points+objects")
    return oaslam::RelocalizationMode::PointsAndObjects;
  if (value == "points") return oaslam::RelocalizationMode::Points;
  throw std::runtime_error(
      "relocalization_mode must be 'points', 'objects', or 'points_and_objects'");
}

/// Write a single pose line in TUM format: timestamp tx ty tz qx qy qz qw
void WriteTumPoseLine(std::ofstream& out, double timestamp,
                      const oaslam::Transform4d& transform) {
  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      rotation(row, col) = transform(row, col);

  Eigen::Quaterniond q(rotation);
  q.normalize();

  out << std::fixed << std::setprecision(6) << timestamp << " "
      << std::setprecision(9)
      << transform(0, 3) << " " << transform(1, 3) << " " << transform(2, 3) << " "
      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
}

constexpr int kMaxKeyframeQueueDepth = 2;
constexpr int kBackpressureSleepUs = 50;

/// Maximum number of recent depth images to keep in the sliding window.
/// Depth images arriving before the window fills are kept; once full the
/// oldest entry is evicted.  10 is generous — at 30 fps depth this covers
/// ~333 ms of depth history.
constexpr size_t kMaxDepthWindowSize = 10;

/// Maximum number of IMU samples to buffer.  At 200 Hz IMU and 30 fps
/// camera this is ~66 s of IMU data — far more than needed between frames.
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
    // ── Bag path (required) ──
    bag_path_ = declare_parameter<std::string>("bag_path", "");
    if (bag_path_.empty()) {
      throw std::runtime_error("'bag_path' parameter is required (path to rosbag directory)");
    }
    if (!std::filesystem::exists(bag_path_)) {
      throw std::runtime_error("Bag path does not exist: " + bag_path_);
    }

    // ── Topic names ──
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>(
        "depth_topic", "/camera/aligned_depth_to_color/image_raw");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/camera/imu");

    // ── Camera / SLAM config ──
    camera_id_ = declare_parameter<std::string>("camera_id", "rgbd0");
    const std::string output_folder = declare_parameter<std::string>("output_folder", "");

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

    // ── Open TUM trajectory file ──
    if (!IsEmptyPath(output_folder)) {
      std::filesystem::create_directories(output_folder);
      const std::string tum_path = output_folder + "/CameraTrajectory.txt";
      tum_file_.open(tum_path, std::ios::out | std::ios::trunc);
      if (!tum_file_.is_open()) {
        throw std::runtime_error("Failed to open TUM trajectory file: " + tum_path);
      }
      tum_file_ << "# TUM trajectory format: timestamp tx ty tz qx qy qz qw\n";
      RCLCPP_INFO(get_logger(), "Saving trajectory (TUM) to: %s", tum_path.c_str());
    }

    // ── Configure SLAM session ──
    oaslam::SessionConfig cfg;
    cfg.slam_backend.vocabulary_file = vocabulary_file;
    cfg.slam_backend.camera_settings_file = camera_settings_file;
    cfg.slam_backend.use_viewer = use_viewer;
    cfg.slam_backend.use_ar_viewer = false;
    cfg.slam_backend.use_objects_in_local_ba = 0;
    cfg.slam_backend.relocalization_mode = ParseRelocalizationMode(relocalization_mode);
    cfg.slam_backend.kind = oaslam::SlamBackendKind::OrbSlam3;
    cfg.slam_backend.use_imu = use_imu;
    cfg.visualizer.enabled = use_viewer;
    cfg.agent_gateway.enabled = false;

    cfg.observation_source.kind = ParseObservationMode(observation_mode);
    cfg.observation_source.ignored_categories =
        LoadIgnoredCategories(ignored_categories_file);

    if (cfg.observation_source.kind == oaslam::ObservationSourceKind::Onnx) {
      if (IsEmptyPath(detection_model_path))
        throw std::runtime_error("detection_model_path required for 'onnx' mode");
      EnsureFileExists(detection_model_path, "ONNX detection model");
      cfg.observation_source.source_path = detection_model_path;
    } else if (cfg.observation_source.kind == oaslam::ObservationSourceKind::File) {
      if (IsEmptyPath(detection_file_path))
        throw std::runtime_error("detection_file_path required for 'file' mode");
      EnsureFileExists(detection_file_path, "Detection file");
      cfg.observation_source.source_path = detection_file_path;
    }

    session_ = std::make_unique<oaslam::SlamSession>(
        cfg, oaslam::CreateDefaultModules(cfg));

    RCLCPP_INFO(get_logger(),
                "OA-SLAM Offline VIO node initialized (ORB-SLAM3, IMU=%s)\n"
                "  Bag:   %s\n"
                "  RGB:   %s\n"
                "  Depth: %s\n"
                "  IMU:   %s",
                use_imu ? "enabled" : "disabled",
                bag_path_.c_str(), rgb_topic_.c_str(),
                depth_topic_.c_str(), imu_topic_.c_str());
  }

  /// Run the full offline pipeline: open bag → stream → process → save.
  void Run() {
    RCLCPP_INFO(get_logger(), "Opening rosbag: %s", bag_path_.c_str());

    // ── Open the bag ──
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = bag_path_;
    storage_opts.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_opts;
    converter_opts.input_serialization_format = "cdr";
    converter_opts.output_serialization_format = "cdr";

    reader.open(storage_opts, converter_opts);

    // ── Collect topic type info ──
    const auto topics_and_types = reader.get_all_topics_and_types();
    std::map<std::string, std::string> topic_type_map;
    for (const auto& info : topics_and_types) {
      topic_type_map[info.name] = info.type;
    }

    // Verify required topics exist
    auto CheckTopic = [&](const std::string& topic, const std::string& label) {
      if (topic_type_map.find(topic) == topic_type_map.end()) {
        RCLCPP_WARN(get_logger(), "%s topic '%s' not found in bag. Available topics:",
                    label.c_str(), topic.c_str());
        for (const auto& [name, type] : topic_type_map) {
          RCLCPP_WARN(get_logger(), "  %s [%s]", name.c_str(), type.c_str());
        }
      }
    };
    CheckTopic(rgb_topic_, "RGB");
    CheckTopic(depth_topic_, "Depth");
    CheckTopic(imu_topic_, "IMU");

    // ── Streaming state ──
    // Instead of loading all messages into memory, we stream them one at a
    // time.  IMU samples are accumulated in a bounded deque, depth images
    // are kept in a small sliding window, and each RGB image triggers
    // immediate frame processing.

    // IMU buffer — bounded deque of deserialized measurements
    std::deque<oaslam::ImuMeasurement> imu_buffer;

    // Depth sliding window — small deque of recent serialized depth messages
    std::deque<TimestampedDepth> depth_window;

    // Counters
    uint64_t frame_counter = 0;
    uint64_t total_rgb_in_bag = 0;
    uint64_t total_frames_processed = 0;
    uint64_t total_poses_obtained = 0;
    uint64_t total_poses_interpolated = 0;
    uint64_t depth_match_failures = 0;
    uint64_t total_imu_in_bag = 0;
    uint64_t total_depth_in_bag = 0;
    uint64_t imu_nan_filtered = 0;

    // Last known good pose — used as fallback for frames where tracking
    // fails, ensuring 1:1 correspondence between image frames and
    // trajectory lines.
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

      // ────────────────────────────────────────────────────────────────
      // IMU message
      // ────────────────────────────────────────────────────────────────
      if (topic == imu_topic_) {
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

        // Evict oldest if buffer exceeds limit (should not happen in
        // normal operation — IMU is drained on every RGB frame)
        while (imu_buffer.size() > kMaxImuBufferSize) {
          imu_buffer.pop_front();
        }

        continue;
      }

      // ────────────────────────────────────────────────────────────────
      // Depth message
      // ────────────────────────────────────────────────────────────────
      if (topic == depth_topic_) {
        total_depth_in_bag++;

        const double ts = NanosToSeconds(bag_msg->time_stamp);
        depth_window.push_back({ts, bag_msg});

        // Evict oldest entries beyond the sliding window size
        while (depth_window.size() > kMaxDepthWindowSize) {
          depth_window.pop_front();
        }

        continue;
      }

      // ────────────────────────────────────────────────────────────────
      // RGB message — triggers frame processing
      // ────────────────────────────────────────────────────────────────
      if (topic == rgb_topic_) {
        total_rgb_in_bag++;

        const double image_timestamp = NanosToSeconds(bag_msg->time_stamp);

        // ── 1. Find closest depth image in the sliding window ──
        constexpr double kMaxDepthMatchSec = 0.1;  // 100ms tolerance
        double best_dt = std::numeric_limits<double>::max();
        size_t best_depth_idx = 0;
        bool found_depth = false;

        for (size_t d = 0; d < depth_window.size(); ++d) {
          double dt = std::abs(depth_window[d].timestamp_sec - image_timestamp);
          if (dt < best_dt) {
            best_dt = dt;
            best_depth_idx = d;
            found_depth = true;
          }
        }

        // ── 2. Deserialize RGB ──
        auto rgb_msg = DeserializeMessage<sensor_msgs::msg::Image>(bag_msg);
        cv_bridge::CvImageConstPtr cv_rgb;
        try {
          cv_rgb = cv_bridge::toCvCopy(rgb_msg, "bgr8");
        } catch (const cv_bridge::Exception& exc) {
          RCLCPP_WARN(get_logger(), "Frame %lu: failed to convert RGB: %s",
                      frame_counter, exc.what());
          // Still write a trajectory line for this frame
          total_frames_processed++;
          total_poses_interpolated++;
          if (tum_file_.is_open()) {
            WriteTumPoseLine(tum_file_, image_timestamp, last_known_pose);
          }
          frame_counter++;
          continue;
        }

        // ── 3. Deserialize depth (if match found within tolerance) ──
        cv::Mat depth_mat;
        bool has_depth = false;
        if (found_depth && best_dt <= kMaxDepthMatchSec) {
          auto depth_msg = DeserializeMessage<sensor_msgs::msg::Image>(
              depth_window[best_depth_idx].msg);
          try {
            auto cv_depth = cv_bridge::toCvCopy(depth_msg);
            depth_mat = cv_depth->image;
            has_depth = true;
          } catch (const cv_bridge::Exception& exc) {
            RCLCPP_WARN(get_logger(), "Frame %lu: failed to convert depth: %s",
                        frame_counter, exc.what());
          }

          // Evict depth entries older than the matched one — they will
          // never be a better match for any future RGB (which has a later
          // timestamp).  Keep the matched entry itself in case the next
          // RGB is very close in time.
          while (depth_window.size() > 1 &&
                 depth_window.front().timestamp_sec <
                     depth_window[best_depth_idx].timestamp_sec) {
            // Recalculate best_depth_idx after pop since indices shift
            if (best_depth_idx > 0) {
              depth_window.pop_front();
              best_depth_idx--;
            } else {
              break;
            }
          }
        } else {
          depth_match_failures++;
          if (depth_match_failures <= 5) {
            RCLCPP_WARN(get_logger(),
                        "Frame %lu: no depth match (best dt=%.3fs). Using empty depth.",
                        frame_counter, best_dt);
          }
        }

        // ── 4. Drain IMU up to image timestamp ──
        std::vector<oaslam::ImuMeasurement> imu_for_frame;
        while (!imu_buffer.empty() &&
               imu_buffer.front().timestamp <= image_timestamp) {
          imu_for_frame.push_back(imu_buffer.front());
          imu_buffer.pop_front();
        }

        // ── 5. Build FramePacket ──
        oaslam::FramePacket frame;
        frame.frame_id = frame_counter++;
        frame.timestamp = image_timestamp;
        frame.camera_id = camera_id_;
        frame.image = cv_rgb->image.clone();
        if (has_depth) {
          frame.depth_image = depth_mat.clone();
          frame.has_depth = true;
        }
        frame.imu_measurements = std::move(imu_for_frame);
        frame.has_imu = !frame.imu_measurements.empty();

        // ── 6. Backpressure ──
        while (session_->keyframesInQueue() > kMaxKeyframeQueueDepth) {
          usleep(kBackpressureSleepUs);
        }

        // ── 7. Process frame ──
        const auto result = session_->processFrame(frame);
        total_frames_processed++;

        // ── 8. Write trajectory line (ALWAYS — 1:1 with RGB images) ──
        if (result.tracking.has_pose) {
          total_poses_obtained++;
          last_known_pose = result.tracking.T_world_camera;
        } else {
          total_poses_interpolated++;
        }

        if (tum_file_.is_open()) {
          WriteTumPoseLine(tum_file_, image_timestamp, last_known_pose);
        }

        // ── 9. Progress logging ──
        if (total_frames_processed % 100 == 0) {
          auto now = std::chrono::steady_clock::now();
          double elapsed_sec = std::chrono::duration<double>(now - wall_start).count();
          double fps = total_frames_processed / elapsed_sec;
          RCLCPP_INFO(get_logger(),
                      "Progress: %lu frames processed, %lu poses, %.1f fps",
                      total_frames_processed,
                      total_poses_obtained, fps);
        }

        // Check if SLAM requested quit
        if (result.quit_requested) {
          RCLCPP_INFO(get_logger(), "SLAM session requested quit.");
          quit_requested = true;
        }

        continue;
      }

      // Other topics are silently ignored.
    }

    // ── Done ──
    auto wall_end = std::chrono::steady_clock::now();
    double total_sec = std::chrono::duration<double>(wall_end - wall_start).count();

    RCLCPP_INFO(get_logger(),
                "\n========================================\n"
                "  Offline VIO processing complete\n"
                "  RGB images in bag: %lu\n"
                "  Depth images in bag: %lu\n"
                "  IMU samples in bag: %lu (NaN filtered: %lu)\n"
                "  Frames processed:  %lu\n"
                "  Poses (tracked):   %lu\n"
                "  Poses (fallback):  %lu\n"
                "  Depth mismatches:  %lu\n"
                "  Trajectory lines:  %lu (1:1 with RGB)\n"
                "  Wall time:         %.1f s\n"
                "  Average FPS:       %.1f\n"
                "========================================",
                total_rgb_in_bag,
                total_depth_in_bag,
                total_imu_in_bag, imu_nan_filtered,
                total_frames_processed, total_poses_obtained,
                total_poses_interpolated, depth_match_failures,
                total_frames_processed,
                total_sec,
                total_frames_processed > 0 ? total_frames_processed / total_sec : 0.0);

    // Flush trajectory file
    if (tum_file_.is_open()) {
      tum_file_.flush();
      tum_file_.close();
      RCLCPP_INFO(get_logger(), "Camera trajectory file saved and closed.");
    }

    // Shutdown SLAM
    session_->shutdown();
    session_.reset();
  }

 private:
  std::unique_ptr<oaslam::SlamSession> session_;
  std::ofstream tum_file_;

  std::string bag_path_;
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string imu_topic_;
  std::string camera_id_;
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
