#ifndef NAV2_CONFIG_HELPER__ROBOT_HPP_
#define NAV2_CONFIG_HELPER__ROBOT_HPP_

#include <nav2_config_helper/object.hpp>

namespace nav2_config_helper
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
    const FootPrint footprint
  );

  std::string getClassName() override {
    return "Robot";
  };

  Object::Type getType() override
  {
    return Type::UNIQUE_MOVABLE;
  }

  std::string getDescription() override
  {
    return "Robot.";
  }

};

} // namespace object

} // namespace nav2_config_helper

#endif