#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <pluginlib/class_loader.hpp>
#include "tf2_ros/transform_broadcaster.h"

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("controller_node");
  rclcpp::Node inner_node("__test_controller_node");
  
  std::string controller_name;
  node->declare_parameter("controller", "dwb_core::DWBLocalPlanner");
  node->get_parameter("controller", controller_name);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap);
  pluginlib::ClassLoader<nav2_core::Controller> loader_("nav2_core", "nav2_core::Controller");
  nav2_core::Controller::Ptr controller;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(inner_node);
  try
  {
    controller = loader_.createSharedInstance(controller_name);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  node->configure();
  costmap->configure();
  controller->configure(node->weak_from_this(), "Controller", costmap->getTfBuffer(), costmap);
  node->activate();
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.header.stamp = node->now();
  t.child_frame_id = costmap->getBaseFrameID();
  tf_broadcaster_->sendTransform(t);
  costmap->activate();
  controller->activate();

  
  while (rclcpp::ok())
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.header.stamp = node->now();
    t.child_frame_id = "base_link";
    tf_broadcaster_->sendTransform(t);
  }
  
  return 0;
}
