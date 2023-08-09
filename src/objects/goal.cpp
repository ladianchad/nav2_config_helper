#include <controller_test/objects/goal.hpp>

namespace controller_test
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
} // namespace controller_test
