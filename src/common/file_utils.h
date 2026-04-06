#ifndef OASLAM_COMMON_FILE_UTILS_H
#define OASLAM_COMMON_FILE_UTILS_H

#include "oaslam/core/session_types.h"

#include <string>
#include <vector>

namespace oaslam {

std::string GetFileExtension(const std::string& path);
bool IsWebcamSource(const std::string& source);
int ParseWebcamId(const std::string& source);
std::vector<int> LoadIgnoredCategories(const std::string& path);
void LoadImageSequenceList(const std::string& path, std::vector<std::string>* image_filenames,
                           std::vector<double>* timestamps);

}  // namespace oaslam

#endif  // OASLAM_COMMON_FILE_UTILS_H
