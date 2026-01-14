#include "rm_ultra_tools/ros2_debugger.hpp"
#include <chrono>
#include <rclcpp/publisher.hpp>

namespace rm_ultra_tools {
ScopedTimeCostMeter::ScopedTimeCostMeter(DoublePubFunc pub, std::string name)
    : pub_(std::move(pub)), name_(std::move(name)),
      start_(std::chrono::steady_clock::now()) {};

ScopedTimeCostMeter::~ScopedTimeCostMeter() {
  using namespace std::chrono;
  double elapsed_ms =
      duration<double, std::milli>(steady_clock::now() - start_).count();
  pub_(name_, elapsed_ms);
}

Debugger::Debugger(rclcpp::Node *node, bool is_debug_)
    : debug_(is_debug_), node_ptr_(node) {}

void Debugger::runIfDebug(std::function<void()> f) const {
  if (this->debug_)
    f();
} // use lamda with & capture to use this fuction

Debugger::DoublePubFunc
Debugger::getLambdaLogPublisher(const std::string &name) {
  if (this->debug_) {
    return DoublePubFunc([this, name](const std::string &topic, double value) {
      this->publishDebugMessage(name, topic, value);
    });
  } else {
    return DoublePubFunc([](const std::string &, double) {});
  }
}
// template <typename T> void Debugger::publish(const std::string &name, T
// &&msg) {
//   auto full_name = this->getFullTopicName(name);
//   auto [has, value] =
//       this->debug_publisher_anymap_.contains<rclcpp::Publisher<T>::SharedPtr>(
//           full_name);
//   auto publisher = // 这是屎
//       has ? value
//           : this->debug_publisher_anymap_
//                 .insert_or_assign<rclcpp::Publisher<T>::SharedPtr>(
//                     full_name,
//                     this->node_ptr_->create_publisher<T>(full_name, 10));
//   publisher->publish(msg);
// }

ScopedTimeCostMeter Debugger::makeScopedTimeCostMeter(const std::string &name) {
  auto time_cost_pub_fn = this->getLambdaLogPublisher("time_cost");
  return ScopedTimeCostMeter{time_cost_pub_fn, name};
}

void Debugger::registTimeCostMeasurement(const std::string &name) {
  std::scoped_lock lock(this->timecost_mutex_);
  this->time_cost_memory_map_.insert_or_assign(name, this->node_ptr_->now());
  return;
}

double Debugger::getTimeCost_ms(const std::string &name) {
  rclcpp::Time start;
  {
    std::scoped_lock lock(this->timecost_mutex_);
    auto it = this->time_cost_memory_map_.find(name);
    if (it == this->time_cost_memory_map_.end()) {
      RCLCPP_WARN(node_ptr_->get_logger(),
                  "Unregistered time key '%s', return 0 ms", name.c_str());
      return 0.0;
    }
    start = it->second;
  } // 锁在这里释放
  return (node_ptr_->now() - start).seconds() * 1000.0;
}

Debugger::ImgPubFunc
Debugger::registImgSubscription(const std::string &img_sub_topic_name,
                                const int &img_update_fps) {
  if (!this->debug_) {
    return [](const std::string &, const ImgProcessFunc &) {};
  } else {
    auto cycle_time = 1.0 / (static_cast<double>(img_update_fps) * 1000.0);
    // 忘记加括号，导致变成了0.06帧
    auto cycle_time_ms = static_cast<int>(cycle_time);
    this->img_cache_update_timer_ = this->node_ptr_->create_wall_timer(
        std::chrono::seconds(cycle_time_ms),
        [this]() { this->img_cache_update_flag_.store(true); });
    auto img_callback = [this, img_sub_topic_name](
                            const sensor_msgs::msg::Image::ConstSharedPtr
                                img_msg) {
      if (this->img_cache_update_flag_.load() == true) {
        auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
        auto img_cached = img.clone();
        {
          std::scoped_lock lock(imgsubpub_mutex_);
          this->subed_img_cache_map_.insert_or_assign( // 没有就新增，有就更新
              img_sub_topic_name,
              std::make_optional(std::make_pair(img_cached, img_msg->header)));
        }
        this->img_cache_update_flag_.store(false);
      }
      return;
    };
    auto img_sub =
        this->node_ptr_->create_subscription<sensor_msgs::msg::Image>(
            img_sub_topic_name, rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
            img_callback);
    this->img_sub_vector_.emplace_back(img_sub); // 要记得保存啊
    return [this, img_sub_topic_name](const std::string &img_pub_topic_name,
                                      const ImgProcessFunc &func) {
      this->editAndPublishSubscribedImg(img_sub_topic_name, img_pub_topic_name,
                                        func);
    };
  }
}

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

void Debugger::putTxt(const std::string &img_pub_topic_name, cv::Mat &img,
                      const cv::Scalar &color, int thickness,
                      cv::Point start_point, int line_spacing) {
  std::vector<std::string> texts;
  {
    std::scoped_lock lock(text_mutex_);
    auto it = this->text_cache_map_.find(img_pub_topic_name);
    if (it == text_cache_map_.end()) {
      return; // 无文本的情况
    } else {
      texts = it->second;
    }
  } // 解锁
  for (const std::string &temp : texts) {
    cv::putText(img, temp, start_point, cv::FONT_HERSHEY_COMPLEX, 0.5, color,
                thickness);
    start_point.y += line_spacing;
  }
}

void Debugger::editAndPublishSubscribedImg(
    const std::string &img_sub_topic_name,
    const std::string &img_pub_topic_name, ImgProcessFunc process_function) {
  cv::Mat img;
  std_msgs::msg::Header header;
  image_transport::Publisher img_pub;
  {
    std::scoped_lock lock(imgsubpub_mutex_);
    auto img_it = this->subed_img_cache_map_.find(img_sub_topic_name);
    if (img_it == subed_img_cache_map_.end()) {
      RCLCPP_WARN(this->node_ptr_->get_logger(),
                  "debugger: The subscribed image is not published!");
      return;
    } else if (img_it->second.has_value() == false) {
      RCLCPP_WARN(this->node_ptr_->get_logger(),
                  "debugger: The subscribed image is empty!");
    } else {
      img_it->second->first.copyTo(img);
      // 也可以使用std::move来搬空img防止竞态，不一定要深拷贝
      // TODO
      // 我草了，太几把狗屎了，从发布端拷贝来copy了一次，这里发布的时候又copy了一次，性能能好才怪
      // 先手动限制下频率凑活着用吧
      header = img_it->second->second;
    }
    auto pub_it = this->img_publisher_map_.find(img_pub_topic_name);
    if (pub_it == img_publisher_map_.end()) {
      auto _img_pub = image_transport::create_publisher(
          this->node_ptr_, this->getFullTopicName(img_pub_topic_name));
      this->img_publisher_map_.insert({img_pub_topic_name, _img_pub});
      img_pub = _img_pub;
    } else {
      img_pub = pub_it->second;
    }
  } // 解锁
  process_function(img);
  this->putTxt(img_pub_topic_name, img);
  this->text_cache_map_.clear();
  img_pub.publish(cv_bridge::CvImage(header, "rgb8", img).toImageMsg());
}

void Debugger::publishDebugMessage(const std::string &name,
                                   const std::string &topic, double value) {
  auto full_topic = name + std::string("/") + topic;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ptr;
  {
    std::scoped_lock lock(this->logpub_mutex_);
    auto it = this->debug_publisher_map_.find(full_topic);
    if (it == debug_publisher_map_.end()) {
      pub_ptr = this->node_ptr_->create_publisher<std_msgs::msg::Float64>(
          this->getFullTopicName(full_topic), 10);
      this->debug_publisher_map_.insert({full_topic, pub_ptr});
    } else {
      pub_ptr = it->second;
    }
  }
  std_msgs::msg::Float64 msg;
  msg.data = value;
  pub_ptr->publish(msg);
}

inline std::string Debugger::getFullTopicName(const std::string &topic) {
  return ("/" + std::string(this->node_ptr_->get_name()) + "/debug" + "/" +
          topic);
}

} // namespace rm_ultra_tools
