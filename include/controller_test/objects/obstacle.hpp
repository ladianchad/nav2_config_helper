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
    const std::shared_ptr<MarkerServer> marker_server,
    const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    std::string frame_id
  );

  std::string getType() override {
    return "Obstacle";
  };
};



} // namespace object

} // namespace controller_test

#endif