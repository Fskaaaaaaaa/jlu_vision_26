#include "yolo.hpp"
#include "configs.hpp"
#include "opencv2/highgui.hpp"
#include "types.hpp"

#include <Eigen/Dense>
#include "quill/LogMacros.h"
#include "opencv2/core/types.hpp"
#include "types/BuffBladeType.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <string>

auto_buff::YOLO::YOLO() {
  generateGridsAndStride();
  config_ = ConfigManager::instance()->configs().yolo_config;
  auto model = core_.read_model(config_.model_path);
  ov::preprocess::PrePostProcessor ppp(model);
  auto &input = ppp.input();
  //Note: 回头来看看这里怎么写
  input.tensor()
      .set_element_type(ov::element::u8)
      .set_shape({1, yolo_input_size, yolo_input_size, 3})
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
  LOG_INFO(ConfigManager::instance()->logger(), "success compile yolov model.");
}

ov::Tensor auto_buff::YOLO::preProcess(const cv::Mat &img) {
  int img_h = img.rows;
  int img_w = img.cols;

  float scale =
      std::min(yolo_input_size * 1.0 / img_h, yolo_input_size * 1.0 / img_w);
  int resize_h = static_cast<int>(round(img_h * scale));
  int resize_w = static_cast<int>(round(img_w * scale));

  int pad_h = yolo_input_size - resize_h;
  int pad_w = yolo_input_size - resize_w;
  float half_h = pad_h * 1.0 / 2;
  float half_w = pad_w * 1.0 / 2;

  int top = static_cast<int>(round(half_h - 0.1));
  int bottom = static_cast<int>(round(half_h + 0.1));
  int left = static_cast<int>(round(half_w - 0.1));
  int right = static_cast<int>(round(half_w + 0.1));

  ov::Tensor input_tensor{ov::element::u8,
                          {1, yolo_input_size, yolo_input_size, 3}};
  cv::Mat input_mat(480, 480, CV_8UC3, input_tensor.data<uint8_t>());

  cv::Mat resized_img;
  cv::resize(img, resized_img, cv::Size(resize_w, resize_h));

  if (!is_recoded_image_parameters_) {
    is_recoded_image_parameters_ = true;
    getTransformMatrix(half_h, half_w, scale);
    input_image_size_ = img.size();
  }

  cv::copyMakeBorder(resized_img, input_mat, top, bottom, left, right,
                     cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  cv::imshow("input", input_mat);
  return input_tensor;
}

void auto_buff::YOLO::getTransformMatrix(float half_h, float half_w,
                                         float scale) {
  /* clang-format off */
  /* *INDENT-OFF* */
  transform_matrix_ << 1.0 / scale, 0          , -half_w / scale,
                       0          , 1.0 / scale, -half_h / scale,
                       0          , 0          , 1;
  /* *INDENT-ON* */
  /* clang-format on */
}
void auto_buff::YOLO::generateGridsAndStride() {
  std::vector<int> strides = {8, 16, 32};
  for (auto stride : strides) {
    int num_grid_w = yolo_input_size / stride;
    int num_grid_h = yolo_input_size / stride;

    for (int g1 = 0; g1 < num_grid_h; g1++) {
      for (int g0 = 0; g0 < num_grid_w; g0++) {
        grid_strides_.push_back({g0, g1, stride});
      }
    }
  }
}

ov::InferRequest auto_buff::YOLO::requestInfer(const ov::Tensor &input_tensor) {
  //这个也是
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  return infer_request;
}

float auto_buff::YOLO::intersectionArea(const RuneObject &a, const RuneObject &b) const {
  cv::Rect_<float> inter = a.box & b.box;
  return inter.area();
}

std::vector<auto_buff::RuneObject>
auto_buff::YOLO::postProcess(const ov::Tensor &output_tensor) {
  auto output_shape = output_tensor.get_shape();
  cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F,
                        output_tensor.data());
  
  std::vector<RuneObject> objs_tmp, objs_result;
  std::vector<int> indices;

  generateProposals(objs_tmp, output_buffer);
  LOG_INFO(ConfigManager::instance()->logger(), "runes size:{}",  std::to_string(objs_tmp.size()));
  std::sort(
      objs_tmp.begin(), objs_tmp.end(),
      [](const RuneObject &a, const RuneObject &b) { return a.prob > b.prob; });
  if (objs_tmp.size() > static_cast<size_t>(config_.top_k)) {
    objs_tmp.resize(config_.top_k);
  }

  nmsMergeSortedBboxes(objs_tmp,indices);

  for (size_t i = 0; i < indices.size(); i++) {
    objs_result.push_back(std::move(objs_tmp[indices[i]]));

    if (objs_result[i].points.children.size() > 0) {
      const float N =
          static_cast<float>(objs_result[i].points.children.size() + 1);
      RunePoints pts_final = std::accumulate(
          objs_result[i].points.children.begin(),
          objs_result[i].points.children.end(), objs_result[i].points);
      objs_result[i].points = pts_final / N;
    }
  }

  return objs_result;
}

void auto_buff::YOLO::generateProposals(std::vector<RuneObject> &output_objs,
                                        const cv::Mat &output_buffer) const {
  float max_confidence = 0;
  for (int anchor_idx = 0; anchor_idx < grid_strides_.size(); anchor_idx++) {
    float confidence =
        output_buffer.at<float>(anchor_idx, yolo_point_number * 2);
    max_confidence = std::max(confidence, max_confidence);
    if (confidence < config_.threshold) {
      continue;
    }

    const int grid0 = grid_strides_[anchor_idx].grid0;
    const int grid1 = grid_strides_[anchor_idx].grid1;
    const int stride = grid_strides_[anchor_idx].stride;

    double color_score, class_score;
    cv::Point color_id, class_id;
    cv::Mat color_scores =
        output_buffer.row(anchor_idx)
            .colRange(yolo_point_number * 2 + 1, yolo_point_number * 2 + 1 + yolo_color_number);
    cv::Mat num_scores =
        output_buffer.row(anchor_idx)
            .colRange(yolo_point_number * 2 + 1 + yolo_color_number,
                      yolo_point_number * 2 + 1 + yolo_color_number + yolo_class_number);
    // Argmax
    cv::minMaxLoc(color_scores, NULL, &color_score, NULL, &color_id);
    cv::minMaxLoc(num_scores, NULL, &class_score, NULL, &class_id);

    float x_1 = (output_buffer.at<float>(anchor_idx, 0) + grid0) * stride;
    float y_1 = (output_buffer.at<float>(anchor_idx, 1) + grid1) * stride;
    float x_2 = (output_buffer.at<float>(anchor_idx, 2) + grid0) * stride;
    float y_2 = (output_buffer.at<float>(anchor_idx, 3) + grid1) * stride;
    float x_3 = (output_buffer.at<float>(anchor_idx, 4) + grid0) * stride;
    float y_3 = (output_buffer.at<float>(anchor_idx, 5) + grid1) * stride;
    float x_4 = (output_buffer.at<float>(anchor_idx, 6) + grid0) * stride;
    float y_4 = (output_buffer.at<float>(anchor_idx, 7) + grid1) * stride;
    float x_5 = (output_buffer.at<float>(anchor_idx, 8) + grid0) * stride;
    float y_5 = (output_buffer.at<float>(anchor_idx, 9) + grid1) * stride;

    Eigen::Matrix<float, 3, 5> apex_norm;
    Eigen::Matrix<float, 3, 5> apex_dst;

    /* clang-format off */
    /* *INDENT-OFF* */
    apex_norm << x_1, x_2, x_3, x_4, x_5,
                y_1, y_2, y_3, y_4, y_5,
                1,   1,   1,   1,   1;
    /* *INDENT-ON* */
    /* clang-format on */

    apex_dst = transform_matrix_ * apex_norm;

    RuneObject obj;

    obj.points.center = cv::Point2f(apex_dst(0, 0), apex_dst(1, 0));
    obj.points.bottom_left = cv::Point2f(apex_dst(0, 1), apex_dst(1, 1));
    obj.points.top_left = cv::Point2f(apex_dst(0, 2), apex_dst(1, 2));
    obj.points.top_right = cv::Point2f(apex_dst(0, 3), apex_dst(1, 3));
    obj.points.bottom_right = cv::Point2f(apex_dst(0, 4), apex_dst(1, 4));

    auto rect = cv::boundingRect(obj.points.toVector2f());

    obj.box = rect;
    obj.color = color_id.x ? types::EnemyColor::Red : types::EnemyColor::Blue;
    obj.type = static_cast<types::BuffBladeType>(class_id.x);
    obj.prob = confidence;

    output_objs.push_back(std::move(obj));
  }
  LOG_INFO(ConfigManager::instance()->logger(), "max_confidence{}",  max_confidence);

}

void auto_buff::YOLO::nmsMergeSortedBboxes(std::vector<RuneObject> &rune_objects,
                                           std::vector<int> &indices) const {
  indices.clear();

  const int object_num = rune_objects.size();

  std::vector<float> areas(object_num);

  for (RuneObject &obj : rune_objects)
    areas.push_back(obj.box.area());

  for (int i = 0; i < object_num; i++) {
    RuneObject &obj_waiting_to_be_merged = rune_objects[i];

    bool keep = true;
    for (int idx : indices) {
      RuneObject &obj_has_been_merged = rune_objects[idx];

      // intersection over union
      float inter_area = intersectionArea(obj_waiting_to_be_merged, obj_has_been_merged);
      float union_area = areas[i] + areas[idx] - inter_area;
      float iou = inter_area / union_area;
      if (iou > config_.nms_threshold || std::isnan(iou)) {
        keep = false;
        // Stored for Merge
        if (obj_waiting_to_be_merged.type == obj_has_been_merged.type && obj_waiting_to_be_merged.color == obj_has_been_merged.color && iou > config_.merge_min_iou &&
            abs(obj_waiting_to_be_merged.prob - obj_has_been_merged.prob) < config_.merge_conf_error) {
          obj_waiting_to_be_merged.points.children.push_back(
              obj_has_been_merged.points);
          obj_has_been_merged.prob = std::max(obj_waiting_to_be_merged.prob, obj_has_been_merged.prob);
        }
      }
    }

    if (keep) {
      indices.push_back(i);
    }
  }
}