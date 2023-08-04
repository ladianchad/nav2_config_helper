#ifndef CONTROLLER_TEST__ROBOT_HPP_
#define CONTROLLER_TEST__ROBOT_HPP_

#include <controller_test/object.hpp>

namespace controller_test
{

namespace object
{

class Robot : public Object
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Robot)
  explicit Robot(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    std::string frame_id,
    std::string footprint_str
  );

  explicit Robot(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    std::string frame_id,
    double footprint_radius
  );

  std::string getType() override {
    return "Robot";
  };

};

} // namespace object

} // namespace controller_test

#endif