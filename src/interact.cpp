#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/transform_datatypes.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <controller_test/interact.hpp>

namespace controller_test
{

namespace interact
{

InteractManager::InteractManager(const rclcpp::Node::SharedPtr node)
{
  this->server_ = std::make_unique<MarkerServer>(
    "basic_controls",
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_topics_interface(),
    node->get_node_services_interface()
  );
}

InteractManager::~InteractManager()
{

}

void
InteractManager::addObject(const object::Object::SharedPtr obj)
{
  visualization_msgs::msg::Marker visible_maker;
  visualization_msgs::msg::InteractiveMarker interact_marker;
  visualization_msgs::msg::InteractiveMarkerControl box_control;
  if(obj->getFootPrintType() == object::Object::FootPrintType::CIRCLE){
    visible_maker.type = visualization_msgs::msg::Marker::SPHERE;
    double size = std::min(obj->getWidth() / 2.0, 1.0);
    visible_maker.scale.x = size;
    visible_maker.scale.y = size;
  } else {
    visible_maker.type = visualization_msgs::msg::Marker::CUBE;
    visible_maker.scale.x = obj->getWidth();
    visible_maker.scale.y = obj->getHeight(); 
  }
  
  visible_maker.scale.z = 0.1;
  visible_maker.pose.position.z = 0.1 / 2.0;
  visible_maker.color = this->getColor(obj->getType());

  // box control
  tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  box_control.orientation = tf2::toMsg(orien);
  box_control.always_visible = true;
  box_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE  ;
  box_control.markers.push_back(visible_maker);
  interact_marker.controls.push_back(box_control);

  // tf2::Quaternion orien;
  visualization_msgs::msg::InteractiveMarkerControl control;
  orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  control.orientation = tf2::toMsg(orien);
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interact_marker.controls.push_back(control);

  interact_marker.header.frame_id = obj->getGlobalFrameId();
  interact_marker.name = obj->getFrameId();
  interact_marker.description = obj->getDescription();
  interact_marker.scale = 1.0;

  this->server_->insert(interact_marker);
  this->server_->setCallback(
    interact_marker.name,
    std::bind(
      &InteractManager::feedbackBackCB,
      this,
      obj,
      std::placeholders::_1
    )
  );
  this->server_->applyChanges();
}

void
InteractManager::feedbackBackCB(
  const object::Object::SharedPtr obj,
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  using Type = visualization_msgs::msg::InteractiveMarkerFeedback::Type;
  switch (feedback->event_type)
  {
  case Type::POSE_UPDATE :
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = feedback->header;
      pose.pose = feedback->pose;
      obj->setPose(pose);
      break;
    }
  case Type::MENU_SELECT:
    break;
  default:
    break;
  }
  this->server_->applyChanges();
}


std_msgs::msg::ColorRGBA
InteractManager::getColor(const object::Object::Type type)
{
  std_msgs::msg::ColorRGBA color;
  auto table_color = InteractManager::ColorMap.find(type)->second;
  color.r = table_color.r;
  color.g = table_color.g;
  color.b = table_color.b;
  color.a = table_color.a;
  return color;
}



} // namespace interact


} // namespace controller_test
