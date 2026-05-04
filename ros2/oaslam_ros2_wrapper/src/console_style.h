#ifndef OASLAM_ROS2_WRAPPER_CONSOLE_STYLE_H
#define OASLAM_ROS2_WRAPPER_CONSOLE_STYLE_H

#include <cstdlib>
#include <string>

namespace oaslam_ros2_wrapper::console {

inline bool ColorEnabled() {
  const char* setting = std::getenv("SCOUTSLAM_COLOR");
  if (setting != nullptr) {
    const std::string value(setting);
    if (value == "0" || value == "false" || value == "never" || value == "off") {
      return false;
    }
    if (value == "1" || value == "true" || value == "always" || value == "on") {
      return true;
    }
  }

  if (std::getenv("NO_COLOR") != nullptr) {
    return false;
  }

  const char* term = std::getenv("TERM");
  return term == nullptr || std::string(term) != "dumb";
}

inline std::string Paint(const std::string& text, const char* code) {
  if (!ColorEnabled()) {
    return text;
  }
  return std::string(code) + text + "\033[0m";
}

inline std::string Muted(const std::string& text) {
  return Paint(text, "\033[2m");
}

inline std::string Blue(const std::string& text) {
  return Paint(text, "\033[34m");
}

inline std::string Prefix(const std::string& label) {
  return Paint("[ScoutSLAM]", "\033[1;36m") + " " + Paint(label, "\033[1;37m");
}

inline std::string StatusOk(const std::string& text) {
  return Paint(text, "\033[1;32m");
}

inline std::string StatusWarn(const std::string& text) {
  return Paint(text, "\033[1;33m");
}

inline std::string StatusError(const std::string& text) {
  return Paint(text, "\033[1;31m");
}

inline std::string KeyValue(const std::string& key, const std::string& value) {
  return Paint(key, "\033[36m") + "=" + Paint(value, "\033[37m");
}

inline std::string OnOff(bool value) {
  return value ? StatusOk("on") : Muted("off");
}

}  // namespace oaslam_ros2_wrapper::console

#endif  // OASLAM_ROS2_WRAPPER_CONSOLE_STYLE_H
