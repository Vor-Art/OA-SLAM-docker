#include "session_config_utils.h"

#include <Eigen/Geometry>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace oaslam_ros2_wrapper {
namespace {

constexpr char kDefaultVocabularyFile[] = "/opt/OA-SLAM/Vocabulary/ORBvoc.txt";
constexpr char kDefaultCameraSettingsFile[] =
    "/opt/OA-SLAM/ros2/oaslam_ros2_wrapper/config/camera/d435i_imu_rgbd.yaml";

std::string FormatCurrentLocalTimestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

  std::tm local_time{};
  localtime_r(&now_time, &local_time);

  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
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
