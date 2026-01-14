// #pragma once
//
// #include "rm_ultra_tools/colors.hpp"
// #include "rm_ultra_tools/thread_safe_any_map.hpp"
//
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.hpp>
// #include <image_transport/publisher.hpp>
// #include <image_transport/subscriber_filter.hpp>
// #include <rclcpp/publisher.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/timer.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <std_msgs/msg/float64.hpp>
//
// #include <opencv2/core.hpp>
//
// #include <atomic>
// #include <chrono>
// #include <functional>
// #include <mutex>
// #include <optional>
// #include <string>
// #include <type_traits>
// #include <utility>
//
// namespace rm_ultra_tools {
//
// class ScopedTimeCostMeter {
// public:
//   using DoublePubFunc = std::function<void(const std::string &, double)>;
//
//   ScopedTimeCostMeter(DoublePubFunc pub, std::string name);
//
//   ~ScopedTimeCostMeter();
//
//   ScopedTimeCostMeter(const ScopedTimeCostMeter &) = delete;
//   ScopedTimeCostMeter &operator=(const ScopedTimeCostMeter &) = delete;
//
// private:
//   DoublePubFunc pub_;
//   std::string name_;
//   std::chrono::steady_clock::time_point start_;
// };
//
// class Debugger {
// public:
//   using ImgSubPtr = rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr;
//   using TopicMap =
//       std::map<std::string,
//                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>;
//   using TimeCostMap = std::map<std::string, rclcpp::Time>;
//   using ImgPubMap = std::map<std::string, image_transport::Publisher>;
//   // using ImgCacheMap =
//   //     std::map<std::string, std::pair<cv::Mat, std_msgs::msg::Header>>;
//   using ImgCacheMap =
//       std::map<std::string,
//                std::optional<std::pair<cv::Mat, std_msgs::msg::Header>>>;
//   using TxtCacheMap = std::map<std::string, std::vector<std::string>>;
//   using ImgProcessFunc = std::function<void(cv::Mat &)>;
//   using DoublePubFunc = std::function<void(const std::string &, double)>;
//   using ImgPubFunc =
//       std::function<void(const std::string &, const ImgProcessFunc &)>;
//
//   Debugger(rclcpp::Node *node, bool is_debug_ = true);
//
//   // template <typename MsgT>
//   // typename rclcpp::Publisher<MsgT>::SharedPtr
//   // getTopicPublisher(std::string topic) {
//   //   return this->debug_ ? this->node_ptr_->create_publisher<MsgT>(
//   //                             this->getFullTopicName(topic), 10)
//   //                       : nullptr;
//   // }
//
//   void runIfDebug(std::function<void()> f)
//       const; // use lamda with & capture to use this fuction
//
//   // std::function<void(const std::string &, double)>
//   DoublePubFunc getLambdaLogPublisher(const std::string &name);
//   template <typename T> void publish(const std::string &name, T &&msg);
//
//   ScopedTimeCostMeter makeScopedTimeCostMeter(const std::string &name);
//
//   void registTimeCostMeasurement(const std::string &name);
//
//   double getTimeCost_ms(const std::string &name);
//
//   ImgPubFunc registImgSubscription(const std::string &img_sub_topic_name,
//                                    const int &img_update_fps = 60);
//
//   template <typename... Args>
//   void addImgTexts(const std::string &img_pub_topic_name, Args &&...args);
//
// private:
//   void putTxt(const std::string &img_pub_topic_name, cv::Mat &img,
//               const cv::Scalar &color = Color::RED, int thickness = 1,
//               cv::Point start_point = cv::Point(0, 50), int line_spacing =
//               30);
//
//   void editAndPublishSubscribedImg(const std::string &img_sub_topic_name,
//                                    const std::string &img_pub_topic_name,
//                                    ImgProcessFunc process_function);
//
//   void publishDebugMessage(const std::string &name, const std::string &topic,
//                            double value);
//
//   inline std::string getFullTopicName(const std::string &topic);
//
//   bool debug_;
//   rclcpp::Node *node_ptr_;
//   AnyMap debug_publisher_anymap_; // 用于模板publish
//   TopicMap debug_publisher_map_;
//   // ^实际上只存储double publisher的，我命名时欠考虑了
//   TimeCostMap time_cost_memory_map_;
//   std::vector<ImgSubPtr> img_sub_vector_;
//   ImgCacheMap subed_img_cache_map_;
//   rclcpp::TimerBase::SharedPtr img_cache_update_timer_;
//   std::atomic<bool> img_cache_update_flag_;
//   ImgPubMap img_publisher_map_;
//   TxtCacheMap text_cache_map_;
//   mutable std::mutex logpub_mutex_, timecost_mutex_, imgsubpub_mutex_,
//       text_mutex_;
// };
//
// template <typename T> void Debugger::publish(const std::string &name, T
// &&msg) {
//   using dT = std::decay_t<T>;
//   using PubT = typename rclcpp::Publisher<dT>::SharedPtr;
//   auto full_name = this->getFullTopicName(name);
//   // auto [has, value] =
//   // this->debug_publisher_anymap_.contains<PubT>(full_name); auto publisher
//   =
//   // // 这是屎
//   //     has ? value
//   //         : this->debug_publisher_anymap_.insert_or_assign<PubT>(
//   //               full_name,
//   //               this->node_ptr_->create_publisher<dT>(full_name, 10));
//   auto publisher_opt = this->debug_publisher_anymap_.find<PubT>(full_name);
//   auto publisher =
//       publisher_opt.has_value()
//           ? *publisher_opt
//           : this->debug_publisher_anymap_
//                 .insert<PubT>(full_name,
//                 this->node_ptr_->create_publisher<dT>(
//                                              full_name, 10))
//                 .first;
//   publisher->publish(msg);
//   return;
// }
//
// template <typename... Args>
// void Debugger::addImgTexts(const std::string &img_pub_topic_name,
//                            Args &&...args) {
//   std::ostringstream oss;
//   ((oss << std::forward<Args>(args) << (sizeof...(Args) > 1 ? " " : "")),
//   ...);
//   {
//     std::scoped_lock lock(text_mutex_);
//     auto it = this->text_cache_map_.find(img_pub_topic_name);
//     if (it == text_cache_map_.end()) {
//       this->text_cache_map_.insert(
//           {img_pub_topic_name, std::vector<std::string>({oss.str()})});
//     } else {
//       it->second.emplace_back(oss.str());
//     }
//   } // 解锁
//   return;
// }
// } // namespace rm_ultra_tools
