/**
* This file is part of OA-SLAM.
*
* Copyright (C) 2022 Matthieu Zins <matthieu.zins@inria.fr>
* (Inria, LORIA, Université de Lorraine)
* OA-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OA-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OA-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <nlohmann/json.hpp>

#include "oaslam/app/module_factories.h"
#include "oaslam/app/slam_session.h"
#include "src/common/file_utils.h"

using json = nlohmann::json;

namespace fs = std::filesystem;

namespace {

oaslam::RelocalizationMode ParseRelocalizationMode(const std::string& value) {
  if (value == "objects") {
    return oaslam::RelocalizationMode::Objects;
  }
  if (value == "points+objects") {
    return oaslam::RelocalizationMode::PointsAndObjects;
  }
  return oaslam::RelocalizationMode::Points;
}

oaslam::ObservationSourceKind ParseObservationSourceKind(const std::string& source) {
  const auto extension = oaslam::GetFileExtension(source);
  if (extension == "json") {
    return oaslam::ObservationSourceKind::File;
  }
  if (extension == "onnx") {
    return oaslam::ObservationSourceKind::Onnx;
  }
  return oaslam::ObservationSourceKind::None;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 10) {
    std::cerr << "\nUsage:\n"
              << " ./oa-slam_localization\n"
              << "      vocabulary_file\n"
              << "      camera_file\n"
              << "      path_to_image_sequence (.txt file listing the images or a folder with rgb.txt)\n"
              << "      detections_file (.json file with detections or .onnx yolov5 weights)\n"
              << "      categories_to_ignore_file\n"
              << "      session_root (directory created by oa-slam)\n"
              << "      relocalization_mode ('points', 'objects' or 'points+objects')\n"
              << "      output_name\n"
              << "      force_relocalization_on_each_frame (0 or 1)\n";
    return 1;
  }

  const std::string vocabulary_file = argv[1];
  const std::string camera_file = argv[2];
  std::string path_to_images = argv[3];
  const std::string detections_file = argv[4];
  const std::string ignored_categories_file = argv[5];
  const fs::path session_root = argv[6];
  const std::string relocalization_mode_value = argv[7];
  const std::string output_name = argv[8];
  const bool force_relocalization = std::stoi(argv[9]) != 0;

  const bool use_webcam = oaslam::IsWebcamSource(path_to_images);
  const int webcam_id = use_webcam ? oaslam::ParseWebcamId(path_to_images) : 0;

  std::string image_list_file = "rgb.txt";
  if (!use_webcam && oaslam::GetFileExtension(path_to_images) == "txt") {
    const auto split = path_to_images.find_last_of('/');
    image_list_file = path_to_images.substr(split + 1);
    path_to_images = path_to_images.substr(0, split + 1);
  }

  if (!use_webcam && !path_to_images.empty() && path_to_images.back() != '/') {
    path_to_images += "/";
  }

  fs::path output_folder = output_name;
  fs::create_directories(output_folder);
  const std::string output_label = 
      output_folder.filename().empty() ? "run" : output_folder.filename().string();


  oaslam::SessionConfig session_config;
  session_config.initial_mode = oaslam::SessionMode::Localization;
  session_config.slam_backend.vocabulary_file = vocabulary_file;
  session_config.slam_backend.camera_settings_file = camera_file;
  session_config.slam_backend.use_viewer = true;
  session_config.slam_backend.use_ar_viewer = false;
  session_config.slam_backend.use_objects_in_local_ba = 0;
  session_config.slam_backend.force_relocalization = force_relocalization;
  session_config.slam_backend.relocalization_mode =
      ParseRelocalizationMode(relocalization_mode_value);
  session_config.observation_source.kind = ParseObservationSourceKind(detections_file);
  session_config.observation_source.source_path = detections_file;
  session_config.observation_source.ignored_categories =
      oaslam::LoadIgnoredCategories(ignored_categories_file);
  session_config.visualizer.enabled = true;
  session_config.agent_gateway.enabled = true;

  oaslam::SlamSession session(session_config, oaslam::CreateDefaultModules(session_config));
  session.loadState(session_root);
  session.setMode(oaslam::SessionMode::Localization);

  cv::VideoCapture capture;
  std::vector<std::string> image_filenames;
  std::vector<double> timestamps;
  std::size_t frame_count = 10000;
  if (!use_webcam) {
    oaslam::LoadImageSequenceList(path_to_images + image_list_file, &image_filenames, &timestamps);
    frame_count = image_filenames.size();
  } else if (!capture.open(webcam_id)) {
    std::cerr << "Failed to open webcam: " << webcam_id << "\n";
    session.shutdown();
    return 1;
  }

  std::vector<float> tracking_times;
  std::vector<double> relocalization_times;
  std::vector<bool> relocalization_status;
  std::vector<cv::Matx44d> poses;
  std::vector<std::string> filenames;
  tracking_times.reserve(frame_count);
  relocalization_times.reserve(frame_count);
  relocalization_status.reserve(frame_count);
  poses.reserve(frame_count);
  filenames.reserve(frame_count);

  for (std::size_t index = 0; index < frame_count; ++index) {
    const auto frame_start = std::chrono::steady_clock::now();

    cv::Mat image;
    std::string filename;
    if (use_webcam) {
      capture >> image;
      filename = "frame_" + std::to_string(index) + ".png";
    } else {
      filename = path_to_images + image_filenames[index];
      image = cv::imread(filename, cv::IMREAD_UNCHANGED);
    }

    if (image.empty()) {
      std::cerr << "Failed to load image: " << filename << "\n";
      session.shutdown();
      return 1;
    }

    const double timestamp =
        index < timestamps.size() ? timestamps[index] : static_cast<double>(std::time(nullptr));
    filenames.push_back(filename);

    oaslam::FramePacket frame;
    frame.frame_id = index;
    frame.timestamp = timestamp;
    frame.camera_id = "mono0";
    frame.image = image;

    const oaslam::SessionFrameResult result = session.processFrame(frame);
    poses.push_back(result.tracking.has_pose ? result.tracking.T_world_camera : cv::Matx44d::eye());
    relocalization_times.push_back(result.tracking.relocalization_duration_ms);
    relocalization_status.push_back(result.tracking.relocalization_success);

    const auto frame_end = std::chrono::steady_clock::now();
    tracking_times.push_back(
        std::chrono::duration_cast<std::chrono::duration<float>>(frame_end - frame_start).count());

    if (result.quit_requested) {
      break;
    }
  }

  json json_data;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const auto& pose = poses[i];
    json rotation({{pose(0, 0), pose(0, 1), pose(0, 2)},
                   {pose(1, 0), pose(1, 1), pose(1, 2)},
                   {pose(2, 0), pose(2, 1), pose(2, 2)}});
    json translation({pose(0, 3), pose(1, 3), pose(2, 3)});
    json_data.push_back({{"file_name", filenames[i]}, {"R", rotation}, {"t", translation}});
  }

  std::ofstream json_file(output_folder / ("camera_poses_" + output_label + ".json"));
  json_file << json_data;

  std::ofstream relocalization_file(output_folder / "relocalization_times.txt");
  for (std::size_t i = 0; i < relocalization_times.size(); ++i) {
    relocalization_file << relocalization_times[i] << " "
                        << static_cast<int>(relocalization_status[i]) << "\n";
  }

  session.shutdown();

  if (!tracking_times.empty()) {
    std::sort(tracking_times.begin(), tracking_times.end());
    float total_time = 0.0F;
    for (const float value : tracking_times) {
      total_time += value;
    }
    std::cout << "median tracking time: " << tracking_times[tracking_times.size() / 2] << "\n";
    std::cout << "mean tracking time: " << total_time / tracking_times.size() << "\n";
  }

  return 0;
}
