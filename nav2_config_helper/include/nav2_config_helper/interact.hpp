#ifndef NAV2_CONFIG_HELPER__INTERACT_HPP_
#define NAV2_CONFIG_HELPER__INTERACT_HPP_

#include <functional>
#include <map>

#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "object.hpp"

namespace nav2_config_helper
{

namespace interact
{

class InteractManager
{
public:
  struct Color
  {
    double r;
    double g;
    double b;
    double a;
  };
  
  const std::map<object::Object::Type, Color> ColorMap = {
    { object::Object::Type::STATIC, { 1.0, 1.0, 1.0, 1.0 } },
    { object::Object::Type::DYNAMIC_NOT_MOVABLE, { 1.0, 0.0, 0.0, 1.0 } },
    { object::Object::Type::DYNAMINC_MOVABLE, { 0.0, 0.0, 1.0, 1.0 } },
    { object::Object::Type::UNIQUE_MOVABLE, { 1.0, 1.0, 1.0, 1.0 } },
    { object::Object::Type::UNIQUE_NOT_MOVABLE, { 0.0, 0.0, 0.0, 1.0 } }
  };

  using MarkerServer = interactive_markers::InteractiveMarkerServer;
  using MarkerPoseCallback = std::function<void(const geometry_msgs::msg::PoseStamped)>;

  RCLCPP_SMART_PTR_DEFINITIONS(InteractManager)
  explicit InteractManager(const rclcpp::Node::SharedPtr node);
  ~InteractManager();

  void addObject(const object::Object::SharedPtr obj);

private:

  std_msgs::msg::ColorRGBA getColor(const object::Object::Type type);

  void feedbackBackCB(
    const object::Object::SharedPtr obj,
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  std::unique_ptr<MarkerServer> server_;
};

} // namespace interact
  
} // namespace nav2_config_helper

#endif