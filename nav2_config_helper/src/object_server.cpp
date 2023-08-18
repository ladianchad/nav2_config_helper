#include <nav2_config_helper/object_server.hpp>

namespace nav2_config_helper
{

namespace object
{

ObjectServer::ObjectServer()
: LifecycleNode("object_server")
{
  this->interact_server_ = std::make_unique<MarkerServer>(
    "object_controls",
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_topics_interface(),
    this->get_node_services_interface()
  );
}

ObjectServer::~ObjectServer()
{}


void
ObjectServer::addObject(Object::ConstSharedPtr obj)
{
  if(this->objs_.count(obj->getId())){
    RCLCPP_ERROR(this->get_logger(), "[id : %ld] %s already registered.", obj->getId(), obj->getClassName());
    return;
  }
  this->objs_[obj->getId()] = obj;
}

void
ObjectServer::removeObject(const object_id_t id)
{
  auto iter = this->objs_.find(id);
  if (iter == this->objs_.end())
  {
    RCLCPP_ERROR(this->get_logger(), "object id %ld not registered.", id);
  }
  this->objs_.erase(this->objs_.find(id));
}

void 
ObjectServer::publishObjects()
{
  for (const auto obj : this->objs_)
  {
    switch (obj.second->getType())
    {
    case Object::Type::GLOBAL_STATIC :
    case Object::Type::GLOBAL_DYNAMIC_NOT_MOVABLE:
    case Object::Type::GLOBAL_DYNAMINC_MOVABLE:
    case Object::Type::GLOBAL_UNIQUE_MOVABLE:
    case Object::Type::GLOBAL_UNIQUE_NOT_MOVABLE:
          
    default:
      break;
    }
  }
  
}


} // namespace object


} // namespace nav2_config_helper
