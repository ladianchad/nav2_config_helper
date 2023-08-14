#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <nav2_config_helper/objects/obstacle.hpp>

namespace nav2_config_helper
{

namespace object
{

Obstacle::Obstacle(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  std::string frame_id
) : Object(node, tf_broad_caster, tf, frame_id)
{
}
} // namespace object
} // namespace nav2_config_helper
