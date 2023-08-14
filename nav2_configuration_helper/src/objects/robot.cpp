#include <nav2_configuration_helper/objects/robot.hpp>

namespace nav2_configuration_helper
{

namespace object
{

Robot::Robot(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  std::string frame_id,
  const FootPrint footprint
) : Object(node, tf_broad_caster, tf, frame_id)
{
  this->frame_id_ = frame_id;
  this->footprint_type_ = FootPrintType::POLYGON;
  this->footprint_ = footprint;
}


} // namespace object
} // namespace nav2_configuration_helper
