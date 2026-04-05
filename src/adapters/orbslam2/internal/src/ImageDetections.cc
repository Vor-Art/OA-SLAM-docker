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


#include "ImageDetections.h"

#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <iterator>
#include <stdexcept>
#include <string_view>


namespace fs = std::filesystem;

namespace {

class DetectionJsonParser {
 public:
  explicit DetectionJsonParser(std::string_view input) : input_(input) {}

  struct ParsedDetection {
    unsigned int category_id = 0;
    double score = 0.0;
    ORB_SLAM2::BBox2 bbox = ORB_SLAM2::BBox2::Zero();
  };

  struct ParsedFrame {
    std::string file_name;
    std::vector<ParsedDetection> detections;
  };

  std::vector<ParsedFrame> parseFrames() {
    skipWhitespace();
    expect('[');

    std::vector<ParsedFrame> frames;
    skipWhitespace();
    if (consume(']')) {
      return frames;
    }

    do {
      frames.push_back(parseFrame());
      skipWhitespace();
    } while (consume(','));

    expect(']');
    skipWhitespace();
    if (!isAtEnd()) {
      throw std::runtime_error("unexpected trailing content");
    }
    return frames;
  }

 private:
  ParsedFrame parseFrame() {
    expect('{');

    ParsedFrame frame;
    skipWhitespace();
    if (consume('}')) {
      return frame;
    }

    do {
      const std::string key = parseString();
      expect(':');

      if (key == "file_name") {
        frame.file_name = parseString();
      } else if (key == "detections") {
        frame.detections = parseDetections();
      } else {
        skipValue();
      }
      skipWhitespace();
    } while (consume(','));

    expect('}');
    return frame;
  }

  std::vector<ParsedDetection> parseDetections() {
    expect('[');

    std::vector<ParsedDetection> detections;
    skipWhitespace();
    if (consume(']')) {
      return detections;
    }

    do {
      detections.push_back(parseDetection());
      skipWhitespace();
    } while (consume(','));

    expect(']');
    return detections;
  }

  ParsedDetection parseDetection() {
    expect('{');

    ParsedDetection detection;
    skipWhitespace();
    if (consume('}')) {
      return detection;
    }

    do {
      const std::string key = parseString();
      expect(':');

      if (key == "bbox") {
        detection.bbox = parseBBox();
      } else if (key == "category_id") {
        detection.category_id = static_cast<unsigned int>(parseNumber());
      } else if (key == "detection_score") {
        detection.score = parseNumber();
      } else {
        skipValue();
      }
      skipWhitespace();
    } while (consume(','));

    expect('}');
    return detection;
  }

  ORB_SLAM2::BBox2 parseBBox() {
    expect('[');

    ORB_SLAM2::BBox2 bbox = ORB_SLAM2::BBox2::Zero();
    for (int i = 0; i < 4; ++i) {
      bbox(i) = parseNumber();
      if (i < 3) {
        expect(',');
      }
    }

    expect(']');
    return bbox;
  }

  std::string parseString() {
    expect('"');

    std::string value;
    while (!isAtEnd()) {
      const char c = input_[position_++];
      if (c == '"') {
        return value;
      }
      if (c == '\\') {
        if (isAtEnd()) {
          throw std::runtime_error("unterminated escape sequence");
        }

        const char escaped = input_[position_++];
        switch (escaped) {
          case '"':
          case '\\':
          case '/':
            value.push_back(escaped);
            break;
          case 'b':
            value.push_back('\b');
            break;
          case 'f':
            value.push_back('\f');
            break;
          case 'n':
            value.push_back('\n');
            break;
          case 'r':
            value.push_back('\r');
            break;
          case 't':
            value.push_back('\t');
            break;
          default:
            throw std::runtime_error("unsupported escaped character");
        }
        continue;
      }

      value.push_back(c);
    }

    throw std::runtime_error("unterminated string");
  }

  double parseNumber() {
    skipWhitespace();
    const char* start = input_.data() + position_;
    char* end = nullptr;
    const double value = std::strtod(start, &end);
    if (end == start) {
      throw std::runtime_error("expected number");
    }

    position_ = static_cast<std::size_t>(end - input_.data());
    return value;
  }

  void skipValue() {
    skipWhitespace();
    if (isAtEnd()) {
      throw std::runtime_error("unexpected end of input");
    }

    const char c = input_[position_];
    if (c == '{') {
      consumeObject();
      return;
    }
    if (c == '[') {
      consumeArray();
      return;
    }
    if (c == '"') {
      parseString();
      return;
    }
    if (std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+') {
      parseNumber();
      return;
    }
    if (consumeLiteral("true") || consumeLiteral("false") || consumeLiteral("null")) {
      return;
    }

    throw std::runtime_error("unsupported JSON value");
  }

  void consumeObject() {
    expect('{');
    skipWhitespace();
    if (consume('}')) {
      return;
    }

    do {
      parseString();
      expect(':');
      skipValue();
      skipWhitespace();
    } while (consume(','));

    expect('}');
  }

  void consumeArray() {
    expect('[');
    skipWhitespace();
    if (consume(']')) {
      return;
    }

    do {
      skipValue();
      skipWhitespace();
    } while (consume(','));

    expect(']');
  }

  bool consumeLiteral(std::string_view literal) {
    skipWhitespace();
    if (input_.substr(position_, literal.size()) != literal) {
      return false;
    }

    position_ += literal.size();
    return true;
  }

  bool consume(char expected) {
    skipWhitespace();
    if (!isAtEnd() && input_[position_] == expected) {
      ++position_;
      return true;
    }
    return false;
  }

  void expect(char expected) {
    skipWhitespace();
    if (isAtEnd() || input_[position_] != expected) {
      throw std::runtime_error(std::string("expected '") + expected + "'");
    }
    ++position_;
  }

  void skipWhitespace() {
    while (!isAtEnd() && std::isspace(static_cast<unsigned char>(input_[position_]))) {
      ++position_;
    }
  }

  bool isAtEnd() const {
    return position_ >= input_.size();
  }

  std::string_view input_;
  std::size_t position_ = 0;
};

}  // namespace


namespace ORB_SLAM2
{

std::ostream& operator <<(std::ostream& os, const Detection& det)
{
    os << "Detection:  cat = " << det.category_id << "  score = "
       << det.score << "  bbox = " << det.bbox.transpose();
    return os;
}


DetectionsFromFile::DetectionsFromFile(const std::string& filename, const std::vector<int>& cats_to_ignore)
    : ImageDetectionsManager()
{
    std::unordered_set<unsigned int> to_ignore(cats_to_ignore.begin(), cats_to_ignore.end());
    std::ifstream fin(filename);
    if (!fin.is_open())
    {
        std::cerr << "Warning failed to open file: " << filename << std::endl;
        return ;
    }

    const std::string contents((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
    try {
        const auto frames = DetectionJsonParser(contents).parseFrames();
        for (const auto& frame : frames)
        {
            std::string name = fs::path(frame.file_name).filename();
            frame_names_.push_back(name);

            std::vector<Detection::Ptr> detections;
            detections.reserve(frame.detections.size());
            for (const auto& det : frame.detections)
            {
                if (to_ignore.find(det.category_id) != to_ignore.end())
                    continue;
                detections.push_back(std::shared_ptr<Detection>(new Detection(det.category_id, det.score, det.bbox)));
            }
            detections_[name] = detections;
        }
    } catch (const std::exception& exc) {
        std::cerr << "Warning failed to parse detections file " << filename << ": "
                  << exc.what() << std::endl;
    }
}


std::vector<Detection::Ptr> DetectionsFromFile::detect(const std::string& name) const {
    std::string basename = fs::path(name).filename();

    if (detections_.find(basename) == detections_.end())
        return {};
    return detections_.at(basename);
}
std::vector<Detection::Ptr> DetectionsFromFile::detect(unsigned int idx) const {
    if (idx >= frame_names_.size()) {
        std::cerr << "Warning invalid index: " << idx << std::endl;
        return {};
    }
    return this->detect(frame_names_[idx]);
}


#ifdef USE_DNN

ObjectDetector::ObjectDetector(const std::string& model, const std::vector<int>& cats_to_ignore) 
    : network_(std::make_unique<cv::dnn::Net>()), ignored_cats_(cats_to_ignore.begin(), cats_to_ignore.end()), ImageDetectionsManager()
{
    if (model.substr(model.size()-4) == "onnx")
        *network_ =  cv::dnn::readNet(model);
    else
        *network_ =  cv::dnn::readNetFromDarknet(model + ".cfg", model + ".weights");
    network_->setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    network_->setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
}

std::vector<Detection::Ptr> ObjectDetector::detect(cv::Mat img) const
{

    // Settings
    const int INPUT_WIDTH = 640.0/2; // size of image passed to the network (reducing may be faster to process)
    const int INPUT_HEIGHT = 640.0/2;
    const float SCORE_THRESHOLD = 0.5;
    const float NMS_THRESHOLD = 0.45;
    const float CONFIDENCE_THRESHOLD = 0.45;

    cv::Mat result;
    cv::dnn::blobFromImage(img, result, 1./255, cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
    
    std::vector<cv::Mat> predictions;
    // for (auto x : network_->getUnconnectedOutLayersNames())
    //     std::cout << x  << std::endl;
    
    auto outnames = network_->getUnconnectedOutLayersNames().back();    // output only the last layer
    network_->setInput(result);
    network_->forward(predictions, outnames);
    const cv::Mat& output = predictions[0];
    // output should have size: 1 x nb_detections x (5 + nb_classes)

    float x_factor = (float)(img.cols) / INPUT_WIDTH;
    float y_factor = (float)(img.rows) / INPUT_HEIGHT;
    float *data = (float *)output.data;

    const int rows = output.size[1];
    const int cols = output.size[2];

    std::vector<int> class_ids;
    class_ids.reserve(1000);
    std::vector<float> confidences;
    confidences.reserve(1000);
    std::vector<cv::Rect> boxes;
    boxes.reserve(1000);
    
    int nb_classes = cols - 5;
    for (int i = 0; i < rows; ++i) {

        float confidence = data[4];
        if (confidence >= .4) {

            float * classes_scores = data + 5;
            cv::Mat scores(1, nb_classes, CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

            if (ignored_cats_.count(class_id.x) == 0 && max_class_score > SCORE_THRESHOLD) {

                confidences.push_back(max_class_score);
                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];
                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                boxes.push_back(cv::Rect(left, top, width, height));    
            }
        }
        data += cols;
    }

    // Filter detection with NMS
    std::vector<Detection::Ptr> detections;
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
    for (int i = 0; i < nms_result.size(); i++) {
        int idx = nms_result[i];
        cv::Rect& bb = boxes[idx];
        Eigen::Vector4d bbox(bb.x, bb.y, bb.x + bb.width, bb.y + bb.height);
        detections.push_back(std::shared_ptr<Detection>(new Detection(class_ids[idx], confidences[idx], bbox)));
    }
    return detections;
}
#endif

}
