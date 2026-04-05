#ifndef OASLAM_APP_SESSION_STATE_IO_H
#define OASLAM_APP_SESSION_STATE_IO_H

#include <filesystem>

#include "oaslam/core/session_types.h"

namespace oaslam {

struct SessionStatePaths {
  std::filesystem::path root;
  std::filesystem::path metadata_dir;
  std::filesystem::path backend_dir;
  std::filesystem::path objects_dir;
  std::filesystem::path session_yaml;
};

SessionStatePaths BuildSessionStatePaths(const std::filesystem::path& root);
void EnsureSessionStateDirectories(const SessionStatePaths& paths);
void WriteSessionMetadata(const SessionStatePaths& paths, const SessionConfig& config,
                          SessionMode mode);

}  // namespace oaslam

#endif  // OASLAM_APP_SESSION_STATE_IO_H
