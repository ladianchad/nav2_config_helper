#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <controller_test/objects/obstacle.hpp>

namespace controller_test
{

namespace object
{

Obstacle::Obstacle(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<MarkerServer> marker_server,
  const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  std::string frame_id
) : Object(node, marker_server, tf_broad_caster, tf, frame_id)
{
}
} // namespace object
} // namespace controller_test
