#ifndef CONTROLLER_TEST__GOAL_HPP_
#define CONTROLLER_TEST__GOAL_HPP_

#include <controller_test/object.hpp>

namespace controller_test
{

namespace object
{

class Goal : public Object
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Goal)

  explicit Goal(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster,
    const std::shared_ptr<tf2_ros::Buffer> tf
  );

  std::string getClassName() override {
    return "Goal";
  };

  Object::Type getType() override
  {
    return Type::UNIQUE_MOVABLE;
  }

  std::string getDescription() override
  {
    return "Goal.";
  }

};

} // namespace object

} // namespace controller_test

#endif