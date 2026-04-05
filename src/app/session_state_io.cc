#include "src/app/session_state_io.h"

#include <fstream>

namespace oaslam {

namespace {

const char* ModeToString(SessionMode mode) {
  return mode == SessionMode::Localization ? "localization" : "mapping";
}

const char* RelocalizationModeToString(RelocalizationMode mode) {
  switch (mode) {
    case RelocalizationMode::Objects:
      return "objects";
    case RelocalizationMode::PointsAndObjects:
      return "points+objects";
    case RelocalizationMode::Points:
    default:
      return "points";
  }
}

}  // namespace

SessionStatePaths BuildSessionStatePaths(const std::filesystem::path& root) {
  SessionStatePaths paths;
  paths.root = root;
  paths.metadata_dir = root / "metadata";
  paths.backend_dir = root / "backend";
  paths.objects_dir = root / "objects";
  paths.session_yaml = paths.metadata_dir / "session.yaml";
  return paths;
}

void EnsureSessionStateDirectories(const SessionStatePaths& paths) {
  std::filesystem::create_directories(paths.metadata_dir);
  std::filesystem::create_directories(paths.backend_dir);
  std::filesystem::create_directories(paths.objects_dir);
}

void WriteSessionMetadata(const SessionStatePaths& paths, const SessionConfig& config,
                          SessionMode mode) {
  EnsureSessionStateDirectories(paths);

  std::ofstream output(paths.session_yaml);
  output << "version: 1\n";
  output << "mode: " << ModeToString(mode) << "\n";
  output << "vocabulary_file: " << config.slam_backend.vocabulary_file << "\n";
  output << "camera_settings_file: " << config.slam_backend.camera_settings_file << "\n";
  output << "relocalization_mode: "
         << RelocalizationModeToString(config.slam_backend.relocalization_mode) << "\n";
  output << "observation_source: " << config.observation_source.source_path << "\n";
}

}  // namespace oaslam
