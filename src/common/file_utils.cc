#include "src/common/file_utils.h"

#include <charconv>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string_view>

namespace oaslam {

std::string GetFileExtension(const std::string& path) {
  const auto idx = path.rfind('.');
  if (idx == std::string::npos) {
    return "";
  }
  return path.substr(idx + 1);
}

bool IsWebcamSource(const std::string& source) {
  return source.size() >= 6 && source.substr(0, 6) == "webcam";
}

int ParseWebcamId(const std::string& source) {
  if (!IsWebcamSource(source)) {
    return 0;
  }

  std::string_view suffix(source.data() + 6, source.size() - 6);
  while (!suffix.empty() && !std::isdigit(static_cast<unsigned char>(suffix.front()))) {
    suffix.remove_prefix(1);
  }
  if (suffix.empty()) {
    return 0;
  }

  int webcam_id = 0;
  const auto [end, error] =
      std::from_chars(suffix.data(), suffix.data() + suffix.size(), webcam_id);
  if (error != std::errc{} || end != suffix.data() + suffix.size()) {
    return 0;
  }
  return webcam_id;
}

std::vector<int> LoadIgnoredCategories(const std::string& path) {
  std::ifstream input(path);
  std::vector<int> categories;
  if (!input.is_open()) {
    return categories;
  }

  int category = 0;
  while (input >> category) {
    categories.push_back(category);
  }
  return categories;
}

void LoadImageSequenceList(const std::string& path,
                           std::vector<std::string>* image_filenames,
                           std::vector<double>* timestamps) {
  std::ifstream input(path);
  std::string line;
  double timestamp = 0.0;
  bool found_timestamps = false;

  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream stream(line);
    std::string image_name;
    if (line.find(' ') != std::string::npos) {
      stream >> timestamp;
      found_timestamps = true;
    }
    stream >> image_name;

    timestamps->push_back(timestamp);
    image_filenames->push_back(image_name);
    if (!found_timestamps) {
      timestamp += 0.033;
    }
  }
}

}  // namespace oaslam
