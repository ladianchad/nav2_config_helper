#ifndef NAV2_CONFIG_HELPER__OBJECT_SERVER_HPP_
#define NAV2_CONFIG_HELPER__OBJECT_SERVER_HPP_


#include <map>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
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

class ObjectServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ObjectServer)
  ObjectServer();
  ~ObjectServer();

private:

  void addObject(Object::ConstSharedPtr obj);

  void removeObject(const object_id_t id);

  std_msgs::msg::ColorRGBA getColor(const Object::Type type);

  void publishObjects();

  void feedbackBackCB(
    const object::Object::SharedPtr obj,
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  std::map<object_id_t, Object::SharedPtr> objs_;

  std::unique_ptr<MarkerServer> interact_server_;
};

} // namespace object


} // namespace nav2_config_helper

#endif