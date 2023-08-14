#include <nav2_configuration_helper/objects/goal.hpp>

namespace nav2_configuration_helper
{

namespace object
{

Goal::Goal(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster,
  const std::shared_ptr<tf2_ros::Buffer> tf
) : Object(node, tf_broad_caster, tf, "goal")
{
}


} // namespace object
} // namespace nav2_configuration_helper
