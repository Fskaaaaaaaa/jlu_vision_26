#pragma once

#include "types.hpp"
#include "configs.hpp"

#include <openvino/openvino.hpp>
#include <Eigen/Dense>
#include <vector>

namespace auto_buff {
class YOLOBase {
public:
  virtual ov::Tensor preProcess(const cv::Mat &image) = 0;
  virtual ov::InferRequest requestInfer(const ov::Tensor &input_tensor) = 0;
  virtual std::vector<RuneObject> postProcess(const ov::Tensor &output_tensor) = 0;
};

class YOLO : public YOLOBase {
public:
  YOLO();
  ~YOLO();
  // NOTE: 返回的tensor是浅拷贝的，并发场景要自己深拷贝下保证生命周期
  ov::Tensor preProcess(const cv::Mat &image) override;
  // NOTE: 只是返回请求，阻塞推理还是并发要自己调用
  ov::InferRequest requestInfer(const ov::Tensor &input_tensor) override;
  std::vector<RuneObject> postProcess(const ov::Tensor &output_tensor) override;

private:
  void generateProposals(
    std::vector<RuneObject> &output_objs,
     const cv::Mat &output_buffer) const;

  void nmsMergeSortedBboxes(std::vector<RuneObject> &rune_objects,
                            std::vector<int> &indices) const;
  
  void getTransformMatrix(float half_h, float half_w, float scale);
  void generateGridsAndStride();
  float intersectionArea(const RuneObject &a, const RuneObject &b) const;

private:
  YOLOConfig config_;
  static constexpr int yolo_input_size = 480;
  static constexpr int yolo_class_number = 2;
  static constexpr int yolo_color_number = 2;
  static constexpr int yolo_point_number = 5;
  
  ov::Core core_;
  ov::CompiledModel compiled_model_;

  bool is_recoded_image_parameters_ = false;
  Eigen::Matrix3f transform_matrix_;
  cv::Size input_image_size_;
  std::vector<GridAndStride> grid_strides_;
};
}