#ifndef CONTROLLER_TEST__INTERACT_HPP_
#define CONTROLLER_TEST__INTERACT_HPP_

#include <functional>

#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "object.hpp"

namespace controller_test
{

namespace interact
{


class InteractManager
{
public:
  using MarkerServer = interactive_markers::InteractiveMarkerServer;
  using MarkerPoseCallback = std::function<void(const geometry_msgs::msg::PoseStamped)>;

  explicit InteractManager(const rclcpp::Node::SharedPtr node);
  ~InteractManager();

  void addObject(const object::Object::SharedPtr obj);

private:
  std::shared_ptr<MarkerServer> server_;
};

} // namespace interact
  
} // namespace controller_test

#endif