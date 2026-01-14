#include "rm_ultra_tools/ros2_rviz2_tools.hpp"
#include <rclcpp/node.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>

namespace rm_ultra_tools {

Rviz2Publisher::Rviz2Publisher(rclcpp::Node *node_ptr, const bool &is_hero) {
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y =
      position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  bullet_marker_.ns = "bullets";
  bullet_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  bullet_marker_.scale.x = bullet_marker_.scale.y = bullet_marker_.scale.z =
      is_hero ? 42.0 / 1000. : 17.0 / 1000.;
  auto name = node_ptr->get_name();
  this->marker_pub_ =
      node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>(
          std::string{name} + "/marker", 10);
}

} // namespace rm_ultra_tools
