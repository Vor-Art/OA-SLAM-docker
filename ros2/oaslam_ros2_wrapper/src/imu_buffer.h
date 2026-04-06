#pragma once

#include <mutex>
#include <deque>
#include <vector>
#include "oaslam/core/frame_packet.h"  // ImuMeasurement

namespace oaslam_ros2 {

class ImuBuffer {
public:
    /// Add a single IMU measurement (called from IMU callback at 200Hz)
    void push(const oaslam::ImuMeasurement& m) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.push_back(m);
        // Keep buffer bounded (e.g., 5 seconds at 200Hz = 1000 samples)
        while (buffer_.size() > 2000) {
            buffer_.pop_front();
        }
    }

    /// Drain all IMU measurements with timestamp <= t_image.
    /// Returns measurements in chronological order.
    /// Called from the image callback to get IMU data between frames.
    std::vector<oaslam::ImuMeasurement> drainUntil(double t_image) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<oaslam::ImuMeasurement> result;
        while (!buffer_.empty() && buffer_.front().timestamp <= t_image) {
            result.push_back(buffer_.front());
            buffer_.pop_front();
        }
        return result;
    }

    /// Get the number of buffered measurements
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.size();
    }

    /// Check if buffer is empty
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.empty();
    }

private:
    mutable std::mutex mutex_;
    std::deque<oaslam::ImuMeasurement> buffer_;
};

}  // namespace oaslam_ros2
