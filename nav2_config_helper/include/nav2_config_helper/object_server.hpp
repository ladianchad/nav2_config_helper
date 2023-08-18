#ifndef NAV2_CONFIG_HELPER__OBJECT_SERVER_HPP_
#define NAV2_CONFIG_HELPER__OBJECT_SERVER_HPP_


#include <vector>

#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "object.hpp"
#include "objects/robot.hpp"
#include "objects/obstacle.hpp"
#include "objects/goal.hpp"

namespace nav2_config_helper
{

namespace object
{

using MarkerServer = interactive_markers::InteractiveMarkerServer;
using MarkerPoseCallback = std::function<void(const geometry_msgs::msg::PoseStamped)>;

class ObjectServer : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ObjectServer)
  ObjectServer(/* args */);
  ~ObjectServer();

private:

  void publishObjects();

  void addObject(Object::ConstSharedPtr obj);

  void removeObject(const object_id_t id);

  std_msgs::msg::ColorRGBA getColor(const Object::Type type);

  void feedbackBackCB(
    const object::Object::SharedPtr obj,
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  std::vector<Object::SharedPtr> objs_;

  std::unique_ptr<MarkerServer> interact_server_;
};

} // namespace object


} // namespace nav2_config_helper

#endif