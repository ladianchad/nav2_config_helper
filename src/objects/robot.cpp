#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <controller_test/objects/robot.hpp>

namespace controller_test
{

namespace object
{

Robot::Robot(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<MarkerServer> marker_server,
  const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  std::string frame_id,
  std::string footprint_str
) : Object(node, marker_server, tf_broad_caster, tf, frame_id)
{
  this->frame_id_ = frame_id;
  this->footprint_type_ = "polygon";
  this->footprint_str_ = footprint_str;
}

Robot::Robot(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<MarkerServer> marker_server,
  const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  std::string frame_id,
  double footprint_radius
) : Object(node, marker_server, tf_broad_caster, tf, frame_id)
{
  this->frame_id_ = frame_id;
  this->footprint_type_ = "circle";
  this->footprint_radius_ = footprint_radius;
}

} // namespace object
} // namespace controller_test
