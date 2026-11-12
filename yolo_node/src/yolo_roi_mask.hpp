// Copyright 2026
// Header-only YOLO ONNX ROI extraction algorithm.
//
// The default output parser supports YOLOv8/YOLO11 raw ONNX output:
//   [1, 4 + classes, anchors] or [1, anchors, 4 + classes]
// For YOLOv5-style output with objectness, set Config::has_objectness = true.
//
// Example:
//   yolo_roi::Config config;
//   config.input_width = 640;
//   config.input_height = 640;
//   yolo_roi::YoloRoiMask masker("model/best.onnx", config);
//   yolo_roi::Result result = masker.process(bgr_image);

#ifndef YOLO_ROI_MASK_HPP_
#define YOLO_ROI_MASK_HPP_

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace yolo_roi
{

struct Detection
{
  int class_id;
  float confidence;
  cv::Rect box;
  std::string class_name;
};

struct Result
{
  // Image with every pixel outside all detected boxes set to zero.
  cv::Mat masked;
  // Original-image crops for every retained detection.
  std::vector<cv::Mat> rois;
  std::vector<Detection> detections;
};

struct Config
{
  int input_width = 640;
  int input_height = 640;
  float score_threshold = 0.25F;
  float nms_threshold = 0.45F;
  bool has_objectness = false;
  std::vector<std::string> class_names;
};

class YoloRoiMask
{
public:
  explicit YoloRoiMask(const std::string & model_path, const Config & config = Config())
  : config_(config)
  {
    if (config_.input_width <= 0 || config_.input_height <= 0) {
      throw std::invalid_argument("YOLO input width and height must be positive");
    }
    if (config_.score_threshold < 0.0F || config_.score_threshold > 1.0F ||
      config_.nms_threshold < 0.0F || config_.nms_threshold > 1.0F)
    {
      throw std::invalid_argument("YOLO score/nms thresholds must be in [0, 1]");
    }

    net_ = cv::dnn::readNetFromONNX(model_path);
    if (net_.empty()) {
      throw std::runtime_error("Failed to load YOLO ONNX model: " + model_path);
    }
  }

  Result process(const cv::Mat & image)
  {
    if (image.empty()) {
      throw std::invalid_argument("Input image is empty");
    }

    cv::Mat bgr;
    if (image.channels() == 3) {
      bgr = image;
    } else if (image.channels() == 1) {
      cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    } else if (image.channels() == 4) {
      cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
    } else {
      throw std::invalid_argument("Unsupported image channel count");
    }

    LetterboxInfo letterbox;
    const cv::Mat input = letterboxImage(bgr, letterbox);

    cv::Mat blob;
    cv::dnn::blobFromImage(
      input, blob, 1.0 / 255.0, cv::Size(config_.input_width, config_.input_height),
      cv::Scalar(), true, false, CV_32F);

    net_.setInput(blob);
    const cv::Mat output = net_.forward();

    std::vector<Detection> candidates = decode(output, letterbox, image.size());
    std::vector<Detection> detections = classWiseNms(candidates);

    Result result;
    result.detections = detections;
    result.masked = cv::Mat::zeros(image.size(), image.type());
    result.rois.reserve(detections.size());

    for (const Detection & detection : detections) {
      const cv::Mat roi = image(detection.box);
      roi.copyTo(result.masked(detection.box));
      result.rois.emplace_back(roi.clone());
    }

    return result;
  }

  const Config & config() const { return config_; }

private:
  struct LetterboxInfo
  {
    float scale = 1.0F;
    int pad_x = 0;
    int pad_y = 0;
  };

  cv::Mat letterboxImage(const cv::Mat & image, LetterboxInfo & info) const
  {
    const float scale_x =
      static_cast<float>(config_.input_width) / static_cast<float>(image.cols);
    const float scale_y =
      static_cast<float>(config_.input_height) / static_cast<float>(image.rows);
    info.scale = std::min(scale_x, scale_y);

    const int resized_width =
      std::max(1, static_cast<int>(std::round(image.cols * info.scale)));
    const int resized_height =
      std::max(1, static_cast<int>(std::round(image.rows * info.scale)));

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(resized_width, resized_height));

    info.pad_x = (config_.input_width - resized_width) / 2;
    info.pad_y = (config_.input_height - resized_height) / 2;

    cv::Mat output;
    cv::copyMakeBorder(
      resized, output, info.pad_y, config_.input_height - resized_height - info.pad_y,
      info.pad_x, config_.input_width - resized_width - info.pad_x, cv::BORDER_CONSTANT,
      cv::Scalar(114, 114, 114));
    return output;
  }

  std::vector<Detection> decode(
    const cv::Mat & output, const LetterboxInfo & letterbox, const cv::Size & image_size) const
  {
    int channels = 0;
    int anchors = 0;
    bool channels_first = false;

    if (output.dims == 3) {
      const int dim_1 = output.size[1];
      const int dim_2 = output.size[2];
      channels_first = dim_1 <= dim_2;
      channels = channels_first ? dim_1 : dim_2;
      anchors = channels_first ? dim_2 : dim_1;
    } else if (output.dims == 2) {
      channels_first = output.rows <= output.cols;
      channels = channels_first ? output.rows : output.cols;
      anchors = channels_first ? output.cols : output.rows;
    } else {
      throw std::runtime_error(
              "Unsupported YOLO output dimensions: " + std::to_string(output.dims));
    }

    if (channels < 5 || anchors <= 0) {
      throw std::runtime_error("Invalid YOLO output shape");
    }

    bool use_objectness = config_.has_objectness;
    if (!config_.class_names.empty()) {
      const int classes = static_cast<int>(config_.class_names.size());
      if (channels == 4 + classes) {
        use_objectness = false;
      } else if (channels == 5 + classes) {
        use_objectness = true;
      }
    }

    const int class_offset = use_objectness ? 5 : 4;
    const int class_count = channels - class_offset;
    if (class_count <= 0) {
      throw std::runtime_error("YOLO output has no class channels");
    }

    cv::Mat contiguous = output.isContinuous() ? output : output.clone();
    const float * data = contiguous.ptr<float>();

    std::vector<Detection> detections;
    detections.reserve(static_cast<std::size_t>(anchors));

    for (int anchor = 0; anchor < anchors; ++anchor) {
      const auto value_at = [&](int channel) -> float {
          const std::size_t index = channels_first
                                      ? static_cast<std::size_t>(channel) * anchors + anchor
                                      : static_cast<std::size_t>(anchor) * channels + channel;
          return data[index];
        };

      const float cx = value_at(0);
      const float cy = value_at(1);
      const float width = value_at(2);
      const float height = value_at(3);
      if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(width) ||
        !std::isfinite(height) || width <= 0.0F || height <= 0.0F)
      {
        continue;
      }

      int class_id = -1;
      float best_class_score = 0.0F;
      for (int class_index = 0; class_index < class_count; ++class_index) {
        const float score = value_at(class_offset + class_index);
        if (score > best_class_score) {
          best_class_score = score;
          class_id = class_index;
        }
      }

      float confidence = best_class_score;
      if (use_objectness) {
        const float objectness = value_at(4);
        if (!std::isfinite(objectness)) {
          continue;
        }
        confidence *= objectness;
      }
      if (class_id < 0 || !std::isfinite(confidence) ||
        confidence < config_.score_threshold)
      {
        continue;
      }

      const float x1 = (cx - width * 0.5F - letterbox.pad_x) / letterbox.scale;
      const float y1 = (cy - height * 0.5F - letterbox.pad_y) / letterbox.scale;
      const float x2 = (cx + width * 0.5F - letterbox.pad_x) / letterbox.scale;
      const float y2 = (cy + height * 0.5F - letterbox.pad_y) / letterbox.scale;

      const int left = std::max(0, std::min(image_size.width - 1, static_cast<int>(std::floor(x1))));
      const int top = std::max(0, std::min(image_size.height - 1, static_cast<int>(std::floor(y1))));
      const int right = std::max(0, std::min(image_size.width, static_cast<int>(std::ceil(x2))));
      const int bottom =
        std::max(0, std::min(image_size.height, static_cast<int>(std::ceil(y2))));
      if (right <= left || bottom <= top) {
        continue;
      }

      Detection detection;
      detection.class_id = class_id;
      detection.confidence = confidence;
      detection.box = cv::Rect(left, top, right - left, bottom - top);
      if (class_id < static_cast<int>(config_.class_names.size())) {
        detection.class_name = config_.class_names[static_cast<std::size_t>(class_id)];
      }
      detections.emplace_back(std::move(detection));
    }

    return detections;
  }

  std::vector<Detection> classWiseNms(const std::vector<Detection> & detections) const
  {
    std::map<int, std::vector<cv::Rect>> boxes_by_class;
    std::map<int, std::vector<float>> scores_by_class;
    std::map<int, std::vector<std::size_t>> indices_by_class;

    for (std::size_t index = 0; index < detections.size(); ++index) {
      const Detection & detection = detections[index];
      boxes_by_class[detection.class_id].push_back(detection.box);
      scores_by_class[detection.class_id].push_back(detection.confidence);
      indices_by_class[detection.class_id].push_back(index);
    }

    std::vector<Detection> kept;
    for (const auto & entry : boxes_by_class) {
      const int class_id = entry.first;
      std::vector<int> selected;
      cv::dnn::NMSBoxes(
        entry.second, scores_by_class[class_id], config_.score_threshold, config_.nms_threshold,
        selected);

      for (const int selected_index : selected) {
        const std::size_t original_index =
          indices_by_class[class_id][static_cast<std::size_t>(selected_index)];
        kept.push_back(detections[original_index]);
      }
    }

    std::sort(
      kept.begin(), kept.end(),
      [](const Detection & lhs, const Detection & rhs) {
        return lhs.confidence > rhs.confidence;
      });
    return kept;
  }

  cv::dnn::Net net_;
  Config config_;
};

}  // namespace yolo_roi

#endif  // YOLO_ROI_MASK_HPP_
