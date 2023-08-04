#ifndef CONTROLLER_TEST__OBJECT_HPP_
#define CONTROLLER_TEST__OBJECT_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <mutex>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav2_util/node_utils.hpp>

namespace controller_test
{

namespace object
{

template<typename NodeT = rclcpp::Node::SharedPtr, typename ParamT>
ParamT getParam(NodeT node, std::string param_name, ParamT default_value) {
  ParamT result;
  nav2_util::declare_parameter_if_not_declared(
    node,
    param_name,
    rclcpp::ParameterValue(default_value)
  );
  node->get_parameter(
    param_name,
    result
  );
  return result;
}

using FootPrint = std::vector<geometry_msgs::msg::Point>;

class Object
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Object)
  explicit Object(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame_id
  );
  ~Object();

  virtual std::string getType() = 0;

  virtual void initialize();

  geometry_msgs::msg::Twist getVelocity() const;

  geometry_msgs::msg::PoseStamped getPose() const;

  void updateTF();

  void setVelocity(const geometry_msgs::msg::Twist &vel = geometry_msgs::msg::Twist());

  void setPose(const geometry_msgs::msg::PoseStamped &pose = geometry_msgs::msg::PoseStamped());

  FootPrint getFootPrint();

protected:
  std::recursive_mutex m_;
  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::Twist velocity_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;

  const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster_;
  const std::shared_ptr<tf2_ros::Buffer> tf_;
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string frame_id_;
  std::string global_frame_id_;
  std::string footprint_type_;
  std::string footprint_str_;
  double footprint_radius_;
};

} // namespace object

} // namespace controller_test


#endif