#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav2_configuration_helper/object.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_util/node_utils.hpp>

namespace nav2_configuration_helper
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

Object::Type
Object::getType()
{
  return Type::DYNAMINC_MOVABLE;
}

std::string
Object::getGlobalFrameId()
{
  return this->global_frame_id_;
}

std::string
Object::getFrameId()
{
  return this->frame_id_;
}

void
Object::initialize() {
  RCLCPP_INFO(this->node_->get_logger(), "Initialize class %s [id : %s]", getClassName().c_str(), getFrameId().c_str());
  global_frame_id_ = getParam(this->node_, getClassName() + ".global_frame_id", std::string("map"));
  this->pose_.header.frame_id = this->global_frame_id_;

  if(!this->footprint_.size()) {
    std::string type;
    type = getParam(this->node_, getClassName() + ".footprint_type", std::string("circle"));
    if(type == "circle"){
      this->footprint_type_ = FootPrintType::CIRCLE;
      double footprint_radius;
      footprint_radius = getParam(this->node_, getClassName() + ".footprint_radius", 0.1);
      this->footprint_ = nav2_costmap_2d::makeFootprintFromRadius(footprint_radius);
    } else if(type == "polygon") {
      this->footprint_type_ = FootPrintType::POLYGON;
      std::string footprint_str;
      footprint_str = getParam(this->node_, getClassName() + ".footprint_str", std::string("[[0.1, 0.1], [-0.1, 0.1], [-0.1, -0.1], [0.1, -0.1]]"));
      nav2_costmap_2d::makeFootprintFromString(footprint_str, this->footprint_);
    } else {
      throw std::runtime_error(getClassName() + ".footprint_type value error");
    }
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
}

FootPrint
Object::getFootPrint()
{
  double theta = tf2::getYaw(this->pose_.pose.orientation);
  FootPrint result;
  nav2_costmap_2d::transformFootprint(
    this->pose_.pose.position.x,
    this->pose_.pose.position.y,
    theta,
    this->footprint_,
    result
  );
  return result;
}

Object::FootPrintType
Object::getFootPrintType()
{
  return this->footprint_type_;
}


double
Object::getWidth()
{
  double width, _;
  nav2_costmap_2d::calculateMinAndMaxDistances(this->footprint_, _, width);
  return width;
}

double
Object::getHeight()
{
  double height, _;
  nav2_costmap_2d::calculateMinAndMaxDistances(this->footprint_, height, _);
  return height;
}

} // namespace object
} // namespace nav2_configuration_helper
