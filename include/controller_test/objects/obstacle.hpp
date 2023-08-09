#ifndef CONTROLLER_TEST__OBSTACLE_HPP_
#define CONTROLLER_TEST__OBSTACLE_HPP_

#include <controller_test/object.hpp>

namespace controller_test
{

namespace object
{

class Obstacle : public Object
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Obstacle)
  explicit Obstacle(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    std::string frame_id
  );

  std::string getClassName() override {
    return "Obstacle";
  };

  std::string getDescription() override
  {
    return "Base obstacle";
  }
};

class DynamicObstacle : public Obstacle
{
  RCLCPP_SMART_PTR_DEFINITIONS(DynamicObstacle)
  Object::Type getType() override {
    return Object::Type::DYNAMINC_MOVABLE;
  }

  std::string getDescription() override
  {
    return "Dynamic obstacle.";
  }
};

class LocalStaticObstacle : public Obstacle
{
  RCLCPP_SMART_PTR_DEFINITIONS(LocalStaticObstacle)
  Object::Type getType() override {
    return Object::Type::DYNAMIC_NOT_MOVABLE;
  }

  std::string getDescription() override
  {
    return "Local static obstacle.";
  }
};

class GlobalStaticObstacle : public Obstacle
{
  RCLCPP_SMART_PTR_DEFINITIONS(GlobalStaticObstacle)
  Object::Type getType() override {
    return Object::Type::STATIC;
  }

  std::string getDescription() override
  {
    return "Global static obstacle.";
  }
};

} // namespace object

} // namespace controller_test

#endif