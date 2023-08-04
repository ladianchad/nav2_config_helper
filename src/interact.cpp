#include <controller_test/interact.hpp>

namespace controller_test
{

namespace interact
{

InteractManager::InteractManager(const rclcpp::Node::SharedPtr node)
{

}

InteractManager::~InteractManager()
{

}

void
InteractManager::addObject(const object::Object::SharedPtr obj)
{
  this->server_->setCallback()
}

} // namespace interact


} // namespace controller_test
