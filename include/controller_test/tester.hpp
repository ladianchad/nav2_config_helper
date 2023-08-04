#ifndef CONTROLLER_TEST__TESTER_HPP_
#define CONTROLLER_TEST__TESTER_HPP_

#include <vector>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/node_thread.hpp>
#include <nav2_core/controller.hpp>
#include <pluginlib/class_loader.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

#include "object.hpp"
#include "objects/robot.hpp"
#include "objects/obstacle.hpp"

namespace controller_test
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using MarkerServer = interactive_markers::InteractiveMarkerServer;

class Tester : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Tester)
  Tester();
  ~Tester();

protected:


  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void run();

  object::Robot * getRobotObject();

  void updateObjects();

  rclcpp::Node::SharedPtr basic_node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<nav2_util::NodeThread> basic_thread_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  nav2_core::Controller::Ptr controller_;

  std::vector<object::Object::SharedPtr> objects_;

  rclcpp::TimerBase::SharedPtr running_timer_;

  std::shared_ptr<MarkerServer> interactive_server_;
};


} // namespace controller_test

#endif