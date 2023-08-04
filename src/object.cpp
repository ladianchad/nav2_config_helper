#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <controller_test/object.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_util/node_utils.hpp>

namespace controller_test
{

namespace object
{
Object::Object(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broad_caster,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame_id
) : tf_broad_caster_(tf_broad_caster), tf_(tf), node_(node), frame_id_(frame_id)
{
}

Object::~Object()
{}

void
Object::initialize() {
  global_frame_id_ = getParam(this->node_, getType() + ".global_frame_id", std::string("map"));
  this->pose_.header.frame_id = this->global_frame_id_;
  footprint_type_ = getParam(this->node_, getType() + ".footprint_type", std::string("circle"));
  if(footprint_type_ == "circle"){
    footprint_radius_ = getParam(this->node_, getType() + ".footprint_radius", 0.1);
  } else if(footprint_type_ == "polygon") {
    footprint_str_ = getParam(this->node_, getType() + ".footprint_str", std::string("[[0.1, 0.1], [-0.1, 0.1], [-0.1, -0.1], [0.1, -0.1]]"));
  } else {
    throw std::runtime_error(getType() + ".footprint_type value error");
  }
  this->pose_sub_ = this->node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->frame_id_ + "/pose",
    10,
    std::bind(&Object::setPose, this, std::placeholders::_1)
  );
  this->vel_sub_ = this->node_->create_subscription<geometry_msgs::msg::Twist>(
    this->frame_id_ + "/vel",
    10,
    std::bind(&Object::setVelocity, this, std::placeholders::_1)
  );
};


geometry_msgs::msg::Twist
Object::getVelocity() const
{
  return this->velocity_;
}

geometry_msgs::msg::PoseStamped 
Object::getPose() const
{
  return this->pose_;
}

void
Object::updateTF()
{
  std::lock_guard<std::recursive_mutex> lock(this->m_);
  this->pose_.header.stamp = this->node_->now();
  geometry_msgs::msg::TransformStamped updated_tf;
  updated_tf.header.frame_id = this->global_frame_id_;
  updated_tf.header = this->pose_.header;
  updated_tf.child_frame_id = this->frame_id_;
  updated_tf.transform.rotation = this->pose_.pose.orientation;
  updated_tf.transform.translation.x = this->pose_.pose.position.x;
  updated_tf.transform.translation.y = this->pose_.pose.position.y;
  updated_tf.transform.translation.z = this->pose_.pose.position.z;
  this->tf_broad_caster_->sendTransform(updated_tf);
}

void
Object::setVelocity(const geometry_msgs::msg::Twist &vel)
{
  std::lock_guard<std::recursive_mutex> lock(this->m_);
  this->velocity_ = vel;
}

void
Object::setPose(const geometry_msgs::msg::PoseStamped &pose)
{
  std::lock_guard<std::recursive_mutex> lock(this->m_);
  this->pose_.pose = pose.pose;
  this->pose_.header.stamp = this->node_->now();
  this->updateTF();
  RCLCPP_INFO(this->node_->get_logger(), "type %s object id: %s updated.", getType().c_str(), this->frame_id_.c_str());
}

FootPrint
Object::getFootPrint()
{
  double theta = tf2::getYaw(this->pose_.pose.orientation);
  FootPrint footprint;
  if(this->footprint_type_ == "circle"){
    footprint = nav2_costmap_2d::makeFootprintFromRadius(this->footprint_radius_);
  } else if (this->footprint_type_ == "polygon") {
    nav2_costmap_2d::makeFootprintFromString(this->footprint_str_, footprint);
  }
  nav2_costmap_2d::transformFootprint(
    this->pose_.pose.position.x,
    this->pose_.pose.position.y,
    theta,
    footprint,
    footprint
  );
  return footprint;
}

} // namespace object
} // namespace controller_test
