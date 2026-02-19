#include "yolo.hpp"

#include "opencv2/core/types.hpp"
#include "quill/LogMacros.h"
#include "types/Armor.hpp"

#include <unordered_map>
#include <vector>

auto_aim::YOLOv5::YOLOv5(quill::Logger *logger, const YOLOConfig &config)
    : logger_(logger), config_(config) {
  auto model = core_.read_model(config_.model_path);
  ov::preprocess::PrePostProcessor ppp(model);
  auto &input = ppp.input();
  input.tensor()
      .set_element_type(ov::element::u8)
      .set_shape({1, 640, 640, 3})
      .set_layout("NHWC")
      .set_color_format(ov::preprocess::ColorFormat::BGR);
  input.model().set_layout("NCHW");
  input.preprocess()
      .convert_element_type(ov::element::f32)
      .convert_color(ov::preprocess::ColorFormat::RGB)
      .scale(255.0);
  model = ppp.build();
  compiled_model_ = core_.compile_model(
      model, config_.device,
      ov::hint::performance_mode(config_.use_latency_performancemode
                                     ? ov::hint::PerformanceMode::LATENCY
                                     : ov::hint::PerformanceMode::THROUGHPUT));
  LOG_INFO(logger_, "success compile yolov5 model.");
}

ov::Tensor auto_aim::YOLOv5::preProcess(const cv::Mat &bgr_image) {
  auto x_scale = static_cast<double>(yolo_input_size) / bgr_image.rows;
  auto y_scale = static_cast<double>(yolo_input_size) / bgr_image.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_image.rows * scale);
  auto w = static_cast<int>(bgr_image.cols * scale);
  cv::Mat input{yolo_input_size, yolo_input_size, CV_8UC3, cv::Scalar(0, 0, 0)};
  cv::Rect roi{0, 0, w, h};
  cv::resize(bgr_image, input(roi), {w, h});
  // NOTE: 填充黑边再缩放成640x640，分辨率降至0.75倍，优先保证视野
  ov::Tensor input_tensor{
      ov::element::u8, {1, yolo_input_size, yolo_input_size, 3}, input.data};
  return input_tensor;
}

ov::InferRequest
auto_aim::YOLOv5::requestInfer(const ov::Tensor &input_tensor) {
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  return infer_request;
}

double auto_aim::YOLOv5::sigmoid(double x) {
  if (x > 0)
    return 1.0 / (1.0 + std::exp(-x));
  else
    return std::exp(x) / (1.0 + std::exp(x));
}

std::vector<auto_aim::Armor>
auto_aim::YOLOv5::postProcess(const ov::Tensor &output_tensor,
                              cv::Size image_size) {
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F,
                 output_tensor.data());
  // for each row: xywh + classess
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::array<cv::Point2f, 4>> armors_key_points;
  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    score = sigmoid(score);
    if (score < config_.score_thresh)
      continue;
    // 颜色和类别独热向量
    cv::Mat color_scores = output.row(r).colRange(9, 13);    // color
    cv::Mat classes_scores = output.row(r).colRange(13, 22); // num
    cv::Point class_id, color_id;
    int class_id_x, color_id_x;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
    class_id_x = class_id.x;
    color_id_x = color_id.x;

    auto x_scale = static_cast<double>(yolo_input_size) / image_size.height;
    auto y_scale = static_cast<double>(yolo_input_size) / image_size.width;
    auto scale = std::min(x_scale, y_scale);
    // NOTE: 1 2
    //       0 3
    //       从济喵的顺序改成君瞄的顺序了
    std::array<cv::Point2f, 4> armor_key_points = {
        cv::Point2f(output.at<float>(r, 2) / scale,
                    output.at<float>(r, 3) / scale),
        cv::Point2f(output.at<float>(r, 0) / scale,
                    output.at<float>(r, 1) / scale),
        cv::Point2f(output.at<float>(r, 6) / scale,
                    output.at<float>(r, 7) / scale),
        cv::Point2f(output.at<float>(r, 4) / scale,
                    output.at<float>(r, 5) / scale),
    };
    // 由于cv的nms没有rretc的，得先打擂台算下外接矩形
    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;
    for (auto &&armor_key_point : armor_key_points) {
      if (armor_key_point.x < min_x)
        min_x = armor_key_point.x;
      if (armor_key_point.x > max_x)
        max_x = armor_key_point.x;
      if (armor_key_point.y < min_y)
        min_y = armor_key_point.y;
      if (armor_key_point.y > max_y)
        max_y = armor_key_point.y;
    }
    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);
    color_ids.emplace_back(color_id_x);
    num_ids.emplace_back(class_id_x);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, config_.score_thresh,
                    config_.nms_iou_thresh, indices);
  std::vector<Armor> armors;
  for (auto &&i : indices) {
    auto confidence = confidences.at(i);
    auto type = type_map_.at(num_ids.at(i));
    if (confidence < config_.accept_thresh ||
        type == types::ArmorType::Negative)
      continue;
    armors.emplace_back(armors_key_points.at(i), confidence, type,
                        color_map_.at(color_ids.at(i)));
  }
  return armors;
}
