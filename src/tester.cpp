#include <controller_test/tester.hpp>

namespace controller_test
{

Tester::Tester()
: LifecycleNode("controller_test"),
  loader_("nav2_core", "nav2_core::Controller")
{
  this->tf_node_ = rclcpp::Node::make_shared(std::string("_") + this->get_name(), this->get_node_options());
  this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->tf_node_);
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  this->costmap_thread_ = std::make_unique<nav2_util::NodeThread>(this->costmap_);
}

Tester::~Tester()
{
  this->costmap_thread_.reset();
}

CallbackReturn 
Tester::on_configure(const rclcpp_lifecycle::State & state)
{
  std::string controller_name;
  this->declare_parameter("controller", "dwb_core::DWBLocalPlanner");
  this->get_parameter("controller", controller_name);
  try
  {
    this->controller_ = loader_.createSharedInstance(controller_name);
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "check controller plugin");
    throw e;
  }
  this->costmap_->configure();
  this->controller_->configure(
    this->weak_from_this(),
    "Controller",
    this->costmap_->getTfBuffer(),
    this->costmap_
  );
  this->objects_.push_back(
    std::make_shared<object::Robot>(
      this->shared_from_this(),
      this->tf_broadcaster_,
      this->tf_buffer_,
      this->costmap_->getBaseFrameID(),
      10
    )
  );
  std::string obstacle_prefix = "obstacle";
  for (size_t i = 0; i < 5; i++)
  {
    this->objects_.push_back(
      std::make_shared<object::Obstacle>(
        this->shared_from_this(),
        this->tf_broadcaster_,
        this->tf_buffer_,
        obstacle_prefix + "_" + std::to_string(i+1)
      )
    );
  }
  
  for (auto &obj: this->objects_)
  {
    obj->initialize();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn
Tester::on_activate(const rclcpp_lifecycle::State & state)
{
  auto robot = getRobotObject();
  robot->updateTF();
  this->costmap_->getCostmap()->resizeMap(100,100, 0.05, -2.5, -2.5);
  this->costmap_->activate();
  this->controller_->activate();
  this->running_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Tester::run, this));  
  return CallbackReturn::SUCCESS;
}

CallbackReturn
Tester::on_deactivate(const rclcpp_lifecycle::State & state)
{
  this->costmap_->deactivate();
  this->controller_->deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
Tester::on_shutdown(const rclcpp_lifecycle::State & state)
{
  this->costmap_->shutdown();
  return CallbackReturn::SUCCESS;
}

void 
Tester::run()
{
  this->updateObjects();
}

object::Robot *
Tester::getRobotObject()
{
  return static_cast<object::Robot *>(this->objects_[0].get());
}

void
Tester::updateObjects()
{
  for (auto &obj : this->objects_)
  {
    obj->updateTF();
  }
}


} // namespace controller_test
