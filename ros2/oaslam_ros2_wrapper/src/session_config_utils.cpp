#include "session_config_utils.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace oaslam_ros2_wrapper {
namespace {

constexpr char kDefaultVocabularyFile[] = "/opt/OA-SLAM/Vocabulary/ORBvoc.txt";
constexpr char kDefaultCameraSettingsFile[] = "";
constexpr int kMaxKeyframeQueueDepth = 2;
constexpr auto kBackpressureSleep = std::chrono::microseconds(50);
constexpr double kMinSemanticAxis = 1e-3;
constexpr double kMinMarkerAxisScale = 0.05;
constexpr double kMinAxisLineWidth = 0.02;
constexpr double kMinLabelHeight = 0.18;

constexpr std::array<std::array<float, 3>, 10> kSemanticPalette{{
    {{0.11F, 0.47F, 0.71F}},  // blue
    {{1.00F, 0.50F, 0.05F}},  // orange
    {{0.17F, 0.63F, 0.17F}},  // green
    {{0.84F, 0.15F, 0.16F}},  // red
    {{0.58F, 0.40F, 0.74F}},  // violet
    {{0.55F, 0.34F, 0.29F}},  // brown
    {{0.89F, 0.47F, 0.76F}},  // pink
    {{0.50F, 0.50F, 0.50F}},  // gray
    {{0.74F, 0.74F, 0.13F}},  // olive
    {{0.09F, 0.75F, 0.81F}},  // cyan
}};

const char* BoolFlag(bool value) {
  return value ? "on" : "off";
}

std_msgs::msg::ColorRGBA MakeColor(float r, float g, float b, float a) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std_msgs::msg::ColorRGBA TintColor(const std_msgs::msg::ColorRGBA& base,
                                   float tint_mix,
                                   float alpha) {
  const float clamped_mix = std::min(std::max(tint_mix, 0.0F), 1.0F);
  return MakeColor(base.r + (1.0F - base.r) * clamped_mix,
                   base.g + (1.0F - base.g) * clamped_mix,
                   base.b + (1.0F - base.b) * clamped_mix,
                   alpha);
}

std_msgs::msg::ColorRGBA SemanticObjectColor(
    const oaslam::SemanticObject& object) {
  const auto& rgb = kSemanticPalette[object.category_id % kSemanticPalette.size()];
  std_msgs::msg::ColorRGBA color = MakeColor(rgb[0], rgb[1], rgb[2], 0.38F);
  switch (object.status) {
    case oaslam::SemanticObjectStatus::InMap:
      color.a = 0.38F;
      break;
    case oaslam::SemanticObjectStatus::Initialized:
      color = TintColor(color, 0.18F, 0.24F);
      break;
    case oaslam::SemanticObjectStatus::Only2D:
      color = TintColor(color, 0.35F, 0.16F);
      break;
    case oaslam::SemanticObjectStatus::Bad:
    default:
      color = MakeColor(0.85F, 0.15F, 0.15F, 0.20F);
      break;
  }
  return color;
}

float AxisAlphaForStatus(oaslam::SemanticObjectStatus status) {
  switch (status) {
    case oaslam::SemanticObjectStatus::InMap:
      return 0.95F;
    case oaslam::SemanticObjectStatus::Initialized:
      return 0.80F;
    case oaslam::SemanticObjectStatus::Only2D:
      return 0.60F;
    case oaslam::SemanticObjectStatus::Bad:
    default:
      return 0.45F;
  }
}

const char* SemanticStatusText(oaslam::SemanticObjectStatus status) {
  switch (status) {
    case oaslam::SemanticObjectStatus::Only2D:
      return "only_2d";
    case oaslam::SemanticObjectStatus::Initialized:
      return "initialized";
    case oaslam::SemanticObjectStatus::InMap:
      return "in_map";
    case oaslam::SemanticObjectStatus::Bad:
    default:
      return "bad";
  }
}

bool IsRenderableSemanticObject(const oaslam::SemanticObject& object) {
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      if (!std::isfinite(object.T_map_object(row, col))) {
        return false;
      }
    }
  }

  return std::isfinite(object.axes[0]) && std::isfinite(object.axes[1]) &&
         std::isfinite(object.axes[2]) &&
         std::max({object.axes[0], object.axes[1], object.axes[2]}) >
             kMinSemanticAxis;
}

double MaxSemanticAxis(const oaslam::SemanticObject& object) {
  return std::max({object.axes[0], object.axes[1], object.axes[2]});
}

std_msgs::msg::Header MarkerHeader(const std_msgs::msg::Header& header,
                                   const PublisherParams& params) {
  std_msgs::msg::Header marker_header = header;
  marker_header.frame_id = params.world_frame_id;
  return marker_header;
}

int32_t MarkerId(std::uint32_t local_object_id) {
  return static_cast<int32_t>(
      std::min<std::uint32_t>(local_object_id,
                              static_cast<std::uint32_t>(
                                  std::numeric_limits<int32_t>::max())));
}

geometry_msgs::msg::Point ToPoint(double x, double y, double z) {
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

geometry_msgs::msg::Point ToPoint(const Eigen::Vector3d& value) {
  return ToPoint(value.x(), value.y(), value.z());
}

Eigen::Matrix3d RotationFromTransform(const oaslam::Transform4d& transform) {
  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation(row, col) = transform(row, col);
    }
  }
  return rotation;
}

std::string FormatCurrentLocalTimestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

  std::tm local_time{};
  localtime_r(&now_time, &local_time);

  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

std::string DefaultSessionId(const rclcpp::Node& node) {
  return std::string(node.get_name()) + "_" + FormatCurrentLocalTimestamp();
}

std::string ResolveOutputFolder(const std::string& output_folder) {
  if (IsEmptyPath(output_folder)) {
    return "";
  }

  return (std::filesystem::path(output_folder) / FormatCurrentLocalTimestamp())
      .string();
}

void EnsureFileExists(const std::string& path, const std::string& label) {
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error(label + " does not exist: " + path);
  }
}

std::vector<int> LoadIgnoredCategoriesFile(const std::string& path) {
  if (IsEmptyPath(path)) {
    return {};
  }

  EnsureFileExists(path, "Ignored categories file");

  std::ifstream input(path);
  std::vector<int> categories;
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

template <size_t N>
std::array<double, N> ReadDoubleArrayParameter(
    rclcpp::Node& node,
    const std::string& name,
    const std::array<double, N>& default_value,
    bool required) {
  const std::vector<double> values =
      node.declare_parameter<std::vector<double>>(
          name, std::vector<double>(default_value.begin(), default_value.end()));

  if (values.size() != N) {
    if (required) {
      std::ostringstream stream;
      stream << name << " must contain exactly " << N << " numbers";
      throw std::runtime_error(stream.str());
    }
    return default_value;
  }

  std::array<double, N> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

void ValidateIntrinsics(const std::array<double, 4>& intrinsics,
                        const std::string& name) {
  if (intrinsics[0] <= 0.0 || intrinsics[1] <= 0.0) {
    throw std::runtime_error(name + " must use positive fx and fy");
  }
}

DepthAlignmentParams DeclareDepthAlignmentParameters(rclcpp::Node& node) {
  DepthAlignmentParams params;
  params.enabled =
      node.declare_parameter<bool>("depth_alignment_enabled", false);
  params.rotation_is_column_major = node.declare_parameter<bool>(
      "depth_alignment_rotation_is_column_major", false);
  params.depth_unit_scale =
      node.declare_parameter<double>("depth_alignment_depth_unit_scale", 0.001);
  params.rgb_intrinsics = ReadDoubleArrayParameter<4>(
      node, "depth_alignment_rgb_intrinsics", params.rgb_intrinsics,
      params.enabled);
  params.depth_intrinsics = ReadDoubleArrayParameter<4>(
      node, "depth_alignment_depth_intrinsics", params.depth_intrinsics,
      params.enabled);
  params.depth_to_rgb_rotation = ReadDoubleArrayParameter<9>(
      node, "depth_alignment_depth_to_rgb_rotation",
      params.depth_to_rgb_rotation, params.enabled);
  params.depth_to_rgb_translation = ReadDoubleArrayParameter<3>(
      node, "depth_alignment_depth_to_rgb_translation",
      params.depth_to_rgb_translation, params.enabled);

  if (params.enabled) {
    if (params.depth_unit_scale <= 0.0) {
      throw std::runtime_error(
          "depth_alignment_depth_unit_scale must be positive");
    }
    ValidateIntrinsics(params.rgb_intrinsics,
                       "depth_alignment_rgb_intrinsics");
    ValidateIntrinsics(params.depth_intrinsics,
                       "depth_alignment_depth_intrinsics");
  }

  return params;
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

}  // namespace

bool IsEmptyPath(const std::string& value) {
  return value.empty() || value == "none" || value == "null";
}

SharedTopicParams DeclareSharedTopicParameters(rclcpp::Node& node) {
  SharedTopicParams params;
  params.rgb_topic =
      node.declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
  params.depth_topic = node.declare_parameter<std::string>(
      "depth_topic", "/camera/aligned_depth_to_color/image_raw");
  params.imu_topic =
      node.declare_parameter<std::string>("imu_topic", "/camera/imu");
  params.camera_id =
      node.declare_parameter<std::string>("camera_id", "rgbd0");
  params.depth_alignment = DeclareDepthAlignmentParameters(node);
  return params;
}

OnlineTopicParams DeclareOnlineTopicParameters(rclcpp::Node& node) {
  OnlineTopicParams params;
  params.shared = DeclareSharedTopicParameters(node);
  params.publisher.pose_topic =
      node.declare_parameter<std::string>("pose_topic", "/oaslam/pose");
  params.publisher.map_points_topic = node.declare_parameter<std::string>(
      "map_points_topic", "/oaslam/map_points");
  params.publisher.new_map_points_topic = node.declare_parameter<std::string>(
      "new_map_points_topic", "/oaslam/new_map_points");
  params.publisher.visible_map_points_topic =
      node.declare_parameter<std::string>(
          "visible_map_points_topic", "/oaslam/visible_map_points");
  params.publisher.semantic_map_snapshot_topic =
      node.declare_parameter<std::string>(
          "semantic_map_snapshot_topic", "/oaslam/semantic_map_snapshot");
  params.publisher.semantic_map_delta_topic = node.declare_parameter<std::string>(
      "semantic_map_delta_topic", "/oaslam/semantic_map_delta");
  params.publisher.semantic_map_markers_topic = node.declare_parameter<std::string>(
      "semantic_map_markers_topic", "/oaslam/semantic_map_markers");
  params.publisher.world_frame_id =
      node.declare_parameter<std::string>("world_frame_id", "map");
  params.publisher.agent_id = node.declare_parameter<std::string>(
      "agent_id",
      params.shared.camera_id.empty() ? std::string("agent0")
                                      : params.shared.camera_id);
  params.publisher.session_id = node.declare_parameter<std::string>(
      "session_id", DefaultSessionId(node));
  params.publisher.local_semantic_map_snapshot_topic =
      node.declare_parameter<std::string>(
          "local_semantic_map_snapshot_topic",
          "/" + params.publisher.agent_id + "/semantic_map_snapshot");
  params.publisher.local_semantic_map_delta_topic =
      node.declare_parameter<std::string>(
          "local_semantic_map_delta_topic",
          "/" + params.publisher.agent_id + "/semantic_map_delta");
  return params;
}

OfflineTopicParams DeclareOfflineTopicParameters(rclcpp::Node& node) {
  OfflineTopicParams params;
  params.shared = DeclareSharedTopicParameters(node);
  params.publisher.pose_topic =
      node.declare_parameter<std::string>("pose_topic", "/oaslam/pose");
  params.publisher.map_points_topic = node.declare_parameter<std::string>(
      "map_points_topic", "/oaslam/map_points");
  params.publisher.new_map_points_topic = node.declare_parameter<std::string>(
      "new_map_points_topic", "/oaslam/new_map_points");
  params.publisher.visible_map_points_topic =
      node.declare_parameter<std::string>(
          "visible_map_points_topic", "/oaslam/visible_map_points");
  params.publisher.semantic_map_snapshot_topic =
      node.declare_parameter<std::string>(
          "semantic_map_snapshot_topic", "/oaslam/semantic_map_snapshot");
  params.publisher.semantic_map_delta_topic = node.declare_parameter<std::string>(
      "semantic_map_delta_topic", "/oaslam/semantic_map_delta");
  params.publisher.semantic_map_markers_topic = node.declare_parameter<std::string>(
      "semantic_map_markers_topic", "/oaslam/semantic_map_markers");
  params.publisher.world_frame_id =
      node.declare_parameter<std::string>("world_frame_id", "map");
  params.publisher.agent_id = node.declare_parameter<std::string>(
      "agent_id",
      params.shared.camera_id.empty() ? std::string("agent0")
                                      : params.shared.camera_id);
  params.publisher.session_id = node.declare_parameter<std::string>(
      "session_id", DefaultSessionId(node));
  params.publisher.local_semantic_map_snapshot_topic =
      node.declare_parameter<std::string>(
          "local_semantic_map_snapshot_topic",
          "/" + params.publisher.agent_id + "/semantic_map_snapshot");
  params.publisher.local_semantic_map_delta_topic =
      node.declare_parameter<std::string>(
          "local_semantic_map_delta_topic",
          "/" + params.publisher.agent_id + "/semantic_map_delta");
  params.bag_path = node.declare_parameter<std::string>("bag_path", "");
  if (params.bag_path.empty()) {
    throw std::runtime_error(
        "'bag_path' parameter is required (path to rosbag directory)");
  }
  if (!std::filesystem::exists(params.bag_path)) {
    throw std::runtime_error("Bag path does not exist: " + params.bag_path);
  }
  params.start_offset_sec =
      node.declare_parameter<double>("start_offset_sec", 0.0);
  if (params.start_offset_sec < 0.0) {
    throw std::runtime_error("start_offset_sec must be non-negative");
  }
  params.finish_time_sec =
      node.declare_parameter<double>("finish_time_sec", -1.0);
  if (params.finish_time_sec >= 0.0 &&
      params.finish_time_sec < params.start_offset_sec) {
    throw std::runtime_error(
        "finish_time_sec must be greater than or equal to start_offset_sec when set");
  }
  return params;
}

CommonSessionParams DeclareCommonSessionParameters(rclcpp::Node& node) {
  CommonSessionParams params;
  params.output_folder = ResolveOutputFolder(
      node.declare_parameter<std::string>("output_folder", ""));
  params.vocabulary_file =
      node.declare_parameter<std::string>("vocabulary_file", kDefaultVocabularyFile);
  params.camera_settings_file = node.declare_parameter<std::string>(
      "camera_settings_file", kDefaultCameraSettingsFile);
  params.observation_mode =
      node.declare_parameter<std::string>("observation_mode", "none");
  params.detection_model_path =
      node.declare_parameter<std::string>("detection_model_path", "");
  params.detection_file_path =
      node.declare_parameter<std::string>("detection_file_path", "");
  params.ignored_categories_file =
      node.declare_parameter<std::string>("ignored_categories_file", "");
  params.relocalization_mode =
      node.declare_parameter<std::string>("relocalization_mode", "points");
  params.use_viewer = node.declare_parameter<bool>("use_viewer", false);
  params.use_imu = node.declare_parameter<bool>("use_imu", true);
  return params;
}

NodeRuntime CreateNodeRuntime(rclcpp::Node& node) {
  NodeRuntime runtime;
  runtime.session_params = DeclareCommonSessionParameters(node);

  const std::string tum_path = OpenTumTrajectoryFile(
      runtime.tum_trajectory_file, runtime.session_params.output_folder);
  if (!tum_path.empty()) {
    RCLCPP_INFO(node.get_logger(), "Trajectory output | tum=%s", tum_path.c_str());
  }

  const oaslam::SessionConfig session_config =
      BuildSessionConfig(runtime.session_params);
  runtime.session = std::make_unique<oaslam::SlamSession>(
      session_config, oaslam::CreateDefaultModules(session_config));
  return runtime;
}

oaslam::SessionConfig BuildSessionConfig(const CommonSessionParams& params) {
  EnsureFileExists(params.vocabulary_file, "Vocabulary file");
  EnsureFileExists(params.camera_settings_file, "Camera settings file");

  oaslam::SessionConfig config;
  config.slam_backend.vocabulary_file = params.vocabulary_file;
  config.slam_backend.camera_settings_file = params.camera_settings_file;
  config.slam_backend.use_viewer = params.use_viewer;
  config.slam_backend.use_ar_viewer = false;
  config.slam_backend.use_objects_in_local_ba = 0;
  config.slam_backend.relocalization_mode =
      ParseRelocalizationMode(params.relocalization_mode);
  config.slam_backend.use_imu = params.use_imu;
  config.visualizer.enabled = params.use_viewer;
  config.agent_gateway.enabled = false;

  config.observation_source.kind = ParseObservationMode(params.observation_mode);
  config.observation_source.ignored_categories =
      LoadIgnoredCategoriesFile(params.ignored_categories_file);

  const oaslam::ModelInputSize model_input_size =
      oaslam::LoadModelInputSize(params.camera_settings_file);
  config.observation_source.model_input_width = model_input_size.width;
  config.observation_source.model_input_height = model_input_size.height;

  if (config.observation_source.kind == oaslam::ObservationSourceKind::Onnx) {
    if (IsEmptyPath(params.detection_model_path)) {
      throw std::runtime_error(
          "detection_model_path is required when observation_mode is 'onnx'");
    }
    EnsureFileExists(params.detection_model_path, "ONNX detection model");
    config.observation_source.source_path = params.detection_model_path;
  } else if (config.observation_source.kind == oaslam::ObservationSourceKind::File) {
    if (IsEmptyPath(params.detection_file_path)) {
      throw std::runtime_error(
          "detection_file_path is required when observation_mode is 'file'");
    }
    EnsureFileExists(params.detection_file_path, "Detection file");
    config.observation_source.source_path = params.detection_file_path;
  }

  return config;
}

std::string OpenTumTrajectoryFile(std::ofstream& output,
                                  const std::string& output_folder) {
  if (IsEmptyPath(output_folder)) {
    return "";
  }

  const std::filesystem::path output_dir(output_folder);
  std::filesystem::create_directories(output_dir);
  const std::filesystem::path output_path = output_dir / "CameraTrajectory.txt";

  output.open(output_path, std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    throw std::runtime_error(
        "Failed to open TUM trajectory file: " + output_path.string());
  }

  output << "# TUM trajectory format: timestamp tx ty tz qx qy qz qw\n";
  return output_path.string();
}

void ShutdownNodeRuntime(NodeRuntime& runtime, const rclcpp::Logger& logger) {
  if (runtime.tum_trajectory_file.is_open()) {
    runtime.tum_trajectory_file.flush();
    runtime.tum_trajectory_file.close();
    RCLCPP_INFO(logger, "Trajectory output closed");
  }

  if (runtime.session) {
    try {
      runtime.session->shutdown();
    } catch (const std::exception& exc) {
      RCLCPP_WARN(logger, "SLAM session shutdown raised | err=%s", exc.what());
    } catch (...) {
      RCLCPP_WARN(logger, "SLAM session shutdown raised | err=unknown");
    }
    runtime.session.reset();
    RCLCPP_INFO(logger, "SLAM session shut down");
  }
}

bool WaitForMappingBackpressure(oaslam::SlamSession& session) {
  while (rclcpp::ok() && session.keyframesInQueue() > kMaxKeyframeQueueDepth) {
    std::this_thread::sleep_for(kBackpressureSleep);
  }
  return rclcpp::ok();
}

void LogNodeStartup(const rclcpp::Logger& logger,
                    const std::string& node_label,
                    const CommonSessionParams& session_params,
                    const std::vector<std::pair<std::string, std::string>>& entries) {
  std::ostringstream stream;
  stream << "OA-SLAM " << node_label
         << " ready | imu=" << BoolFlag(session_params.use_imu)
         << " viewer=" << BoolFlag(session_params.use_viewer)
         << " obs=" << session_params.observation_mode
         << " relocal=" << session_params.relocalization_mode;
  if (!entries.empty()) {
    stream << "\n  ";
  }
  for (size_t i = 0; i < entries.size(); ++i) {
    if (i > 0) {
      stream << " | ";
    }
    stream << entries[i].first << "=" << entries[i].second;
  }
  RCLCPP_INFO(logger, "%s", stream.str().c_str());
}

geometry_msgs::msg::PoseStamped ToPoseStamped(
    const std_msgs::msg::Header& header,
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

geometry_msgs::msg::Pose ToPose(const oaslam::Transform4d& transform) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = transform(2, 3);

  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation(row, col) = transform(row, col);
    }
  }

  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

sensor_msgs::msg::PointCloud2 ToPointCloud2(
    const std_msgs::msg::Header& header,
    const std::string& world_frame_id,
    const std::vector<cv::Point3d>& points) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.header.frame_id = world_frame_id;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (const auto& point : points) {
    *iter_x = static_cast<float>(point.x);
    *iter_y = static_cast<float>(point.y);
    *iter_z = static_cast<float>(point.z);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return cloud;
}

msg::SemanticObject ToSemanticObjectMsg(
    const oaslam::SemanticObject& object) {
  msg::SemanticObject message;
  message.local_object_id = object.local_object_id;
  message.category_id = object.category_id;
  message.status = static_cast<std::uint8_t>(object.status);
  message.pose = ToPose(object.T_map_object);
  message.axes.x = object.axes[0];
  message.axes.y = object.axes[1];
  message.axes.z = object.axes[2];
  message.observation_count = object.observation_count;
  message.keyframe_observation_count = object.keyframe_observation_count;
  message.last_obs_score = object.last_obs_score;
  message.last_obs_frame_id = object.last_obs_frame_id;
  return message;
}

msg::SemanticMapSnapshot ToSemanticMapSnapshotMsg(
    const std_msgs::msg::Header& header,
    const PublisherParams& params,
    const oaslam::SemanticMapSnapshot& snapshot) {
  msg::SemanticMapSnapshot message;
  message.header = header;
  message.header.frame_id = params.world_frame_id;
  message.agent_id = params.agent_id;
  message.session_id = params.session_id;
  message.map_id = snapshot.map_id;
  message.sequence = snapshot.sequence;
  message.objects.reserve(snapshot.objects.size());
  for (const auto& object : snapshot.objects) {
    message.objects.push_back(ToSemanticObjectMsg(object));
  }
  return message;
}

msg::SemanticMapDelta ToSemanticMapDeltaMsg(
    const std_msgs::msg::Header& header,
    const PublisherParams& params,
    const oaslam::SemanticMapDelta& delta) {
  msg::SemanticMapDelta message;
  message.header = header;
  message.header.frame_id = params.world_frame_id;
  message.agent_id = params.agent_id;
  message.session_id = params.session_id;
  message.map_id = delta.map_id;
  message.sequence = delta.sequence;
  message.reset = delta.reset;
  message.added.reserve(delta.added.size());
  for (const auto& object : delta.added) {
    message.added.push_back(ToSemanticObjectMsg(object));
  }
  message.updated.reserve(delta.updated.size());
  for (const auto& object : delta.updated) {
    message.updated.push_back(ToSemanticObjectMsg(object));
  }
  message.removed_object_ids = delta.removed_object_ids;
  return message;
}

shared_semantic_map_interfaces::msg::LocalSemanticObject
ToLocalSemanticObjectMsg(const oaslam::SemanticObject& object) {
  shared_semantic_map_interfaces::msg::LocalSemanticObject message;
  message.local_object_id = object.local_object_id;
  message.category_id = object.category_id;
  message.status = static_cast<std::uint8_t>(object.status);
  message.pose = ToPose(object.T_map_object);
  message.axes.x = object.axes[0];
  message.axes.y = object.axes[1];
  message.axes.z = object.axes[2];
  message.observation_count = object.observation_count;
  message.keyframe_observation_count = object.keyframe_observation_count;
  message.last_obs_score = object.last_obs_score;
  message.last_obs_frame_id = object.last_obs_frame_id;
  return message;
}

shared_semantic_map_interfaces::msg::LocalSemanticMapSnapshot
ToLocalSemanticMapSnapshotMsg(const std_msgs::msg::Header& header,
                              const PublisherParams& params,
                              const oaslam::SemanticMapSnapshot& snapshot) {
  shared_semantic_map_interfaces::msg::LocalSemanticMapSnapshot message;
  message.header = header;
  message.header.frame_id = params.world_frame_id;
  message.agent_id = params.agent_id;
  message.session_id = params.session_id;
  message.map_id = snapshot.map_id;
  message.sequence = snapshot.sequence;
  message.objects.reserve(snapshot.objects.size());
  for (const auto& object : snapshot.objects) {
    message.objects.push_back(ToLocalSemanticObjectMsg(object));
  }
  return message;
}

shared_semantic_map_interfaces::msg::LocalSemanticMapDelta
ToLocalSemanticMapDeltaMsg(const std_msgs::msg::Header& header,
                           const PublisherParams& params,
                           const oaslam::SemanticMapDelta& delta) {
  shared_semantic_map_interfaces::msg::LocalSemanticMapDelta message;
  message.header = header;
  message.header.frame_id = params.world_frame_id;
  message.agent_id = params.agent_id;
  message.session_id = params.session_id;
  message.map_id = delta.map_id;
  message.sequence = delta.sequence;
  message.reset = delta.reset;
  message.added.reserve(delta.added.size());
  for (const auto& object : delta.added) {
    message.added.push_back(ToLocalSemanticObjectMsg(object));
  }
  message.updated.reserve(delta.updated.size());
  for (const auto& object : delta.updated) {
    message.updated.push_back(ToLocalSemanticObjectMsg(object));
  }
  message.removed_object_ids = delta.removed_object_ids;
  return message;
}

visualization_msgs::msg::MarkerArray ToSemanticMapMarkers(
    const std_msgs::msg::Header& header,
    const PublisherParams& params,
    const oaslam::SemanticMapSnapshot& snapshot) {
  visualization_msgs::msg::MarkerArray markers;
  const std_msgs::msg::Header marker_header = MarkerHeader(header, params);

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header = marker_header;
  clear_marker.ns = "semantic_map";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear_marker);

  for (const auto& object : snapshot.objects) {
    if (!IsRenderableSemanticObject(object)) {
      continue;
    }

    const int32_t marker_id = MarkerId(object.local_object_id);
    const double max_axis = MaxSemanticAxis(object);
    const std_msgs::msg::ColorRGBA object_color = SemanticObjectColor(object);

    visualization_msgs::msg::Marker ellipsoid_marker;
    ellipsoid_marker.header = marker_header;
    ellipsoid_marker.ns = "semantic_map/ellipsoids";
    ellipsoid_marker.id = marker_id;
    ellipsoid_marker.type = visualization_msgs::msg::Marker::SPHERE;
    ellipsoid_marker.action = visualization_msgs::msg::Marker::ADD;
    ellipsoid_marker.pose = ToPose(object.T_map_object);
    ellipsoid_marker.scale.x = std::max(object.axes[0] * 2.0, kMinMarkerAxisScale);
    ellipsoid_marker.scale.y = std::max(object.axes[1] * 2.0, kMinMarkerAxisScale);
    ellipsoid_marker.scale.z = std::max(object.axes[2] * 2.0, kMinMarkerAxisScale);
    ellipsoid_marker.color = object_color;
    markers.markers.push_back(ellipsoid_marker);

    visualization_msgs::msg::Marker axes_marker;
    axes_marker.header = marker_header;
    axes_marker.ns = "semantic_map/axes";
    axes_marker.id = marker_id;
    axes_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    axes_marker.action = visualization_msgs::msg::Marker::ADD;
    axes_marker.pose.orientation.w = 1.0;
    axes_marker.scale.x = std::max(kMinAxisLineWidth, max_axis * 0.08);

    const Eigen::Matrix3d rotation = RotationFromTransform(object.T_map_object);
    const Eigen::Vector3d center(object.T_map_object(0, 3),
                                 object.T_map_object(1, 3),
                                 object.T_map_object(2, 3));
    const std::array<Eigen::Vector3d, 3> local_axes{
        Eigen::Vector3d(object.axes[0], 0.0, 0.0),
        Eigen::Vector3d(0.0, object.axes[1], 0.0),
        Eigen::Vector3d(0.0, 0.0, object.axes[2]),
    };
    const float axis_alpha = AxisAlphaForStatus(object.status);
    const std::array<std_msgs::msg::ColorRGBA, 3> axis_colors{
        MakeColor(0.95F, 0.25F, 0.25F, axis_alpha),
        MakeColor(0.20F, 0.85F, 0.35F, axis_alpha),
        MakeColor(0.25F, 0.55F, 0.98F, axis_alpha),
    };

    axes_marker.points.reserve(6);
    axes_marker.colors.reserve(6);
    for (size_t axis_index = 0; axis_index < local_axes.size(); ++axis_index) {
      const Eigen::Vector3d world_axis = rotation * local_axes[axis_index];
      axes_marker.points.push_back(ToPoint(center - world_axis));
      axes_marker.points.push_back(ToPoint(center + world_axis));
      axes_marker.colors.push_back(axis_colors[axis_index]);
      axes_marker.colors.push_back(axis_colors[axis_index]);
    }
    markers.markers.push_back(axes_marker);

    visualization_msgs::msg::Marker label_marker;
    label_marker.header = marker_header;
    label_marker.ns = "semantic_map/labels";
    label_marker.id = marker_id;
    label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label_marker.action = visualization_msgs::msg::Marker::ADD;
    label_marker.pose.position = ToPoint(center.x(), center.y(),
                                         center.z() + max_axis + 0.20);
    label_marker.pose.orientation.w = 1.0;
    label_marker.scale.z = std::max(kMinLabelHeight, max_axis * 0.35);
    label_marker.color = TintColor(object_color, 0.72F, 0.96F);

    std::ostringstream text;
    text << "#" << object.local_object_id
         << " cat:" << object.category_id
         << " " << SemanticStatusText(object.status)
         << " obs:" << object.observation_count;
    label_marker.text = text.str();
    markers.markers.push_back(label_marker);
  }

  return markers;
}

void WriteTumPoseLine(std::ofstream& output,
                      double timestamp,
                      const oaslam::Transform4d& transform) {
  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation(row, col) = transform(row, col);
    }
  }

  Eigen::Quaterniond q(rotation);
  q.normalize();

  output << std::fixed << std::setprecision(6) << timestamp << " "
         << std::setprecision(9)
         << transform(0, 3) << " " << transform(1, 3) << " "
         << transform(2, 3) << " " << q.x() << " " << q.y() << " "
         << q.z() << " " << q.w() << "\n";
}

}  // namespace oaslam_ros2_wrapper
