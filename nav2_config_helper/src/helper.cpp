#include <nav2_config_helper/helper.hpp>

#include <nav2_config_helper/objects/obstacle.hpp>

namespace nav2_config_helper
{

Helper::Helper(
  const rclcpp::executors::Executor::SharedPtr & executor
)
: LifecycleNode("nav2_config_helper"),
  loader_("nav2_core", "nav2_core::Controller")
{
  this->executor_ = executor;
  this->basic_node_ = rclcpp::Node::make_shared(std::string("_") + this->get_name(), this->get_node_options());
  this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->basic_node_);
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  this->interact_manger_ = std::make_unique<interact::InteractManager>(this->basic_node_);
  this->costmap_thread_ = std::make_unique<nav2_util::NodeThread>(this->costmap_);
  this->basic_thread_ = std::make_unique<nav2_util::NodeThread>(this->basic_node_);
  this->test_layer_ = std::make_shared<layer::TestLayer>();
  this->vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
}

Helper::~Helper()
{
  this->costmap_thread_.reset();
  this->basic_thread_.reset();
}

CallbackReturn 
Helper::on_configure(const rclcpp_lifecycle::State & state)
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
      this->costmap_->getUnpaddedRobotFootprint()
    )
  );
  this->objects_.push_back(
    std::make_shared<object::Goal>(
      this->shared_from_this(),
      this->tf_broadcaster_,
      this->tf_buffer_
    )
  );
  std::string obstacle_prefix = "obstacle";
  for (size_t i = 0; i < 1; i++)
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
    this->interact_manger_->addObject(obj);
  }
  this->test_layer_->initialize(
    this->costmap_->getLayeredCostmap(),
    "TestLayer",
    this->costmap_->getTfBuffer().get(),
    this->weak_from_this(),
    nullptr
  );
  auto plugins = this->costmap_->getLayeredCostmap()->getPlugins();
  plugins->insert(plugins->begin(), this->test_layer_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn
Helper::on_activate(const rclcpp_lifecycle::State & state)
{
  this->vel_pub_->on_activate();
  auto robot = getRobotObject();
  robot->updateTF();
  this->costmap_->getCostmap()->resizeMap(100,100, 0.05, -2.5, -2.5);
  this->costmap_->activate();
  this->controller_->activate();
  this->running_timer_ = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&Helper::run, this));
  this->update_timer_ = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&Helper::updateObjects, this));
  return CallbackReturn::SUCCESS;
}

CallbackReturn
Helper::on_deactivate(const rclcpp_lifecycle::State & state)
{
  this->vel_pub_->on_deactivate();
  this->costmap_->deactivate();
  this->controller_->deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
Helper::on_shutdown(const rclcpp_lifecycle::State & state)
{
  this->costmap_->shutdown();
  return CallbackReturn::SUCCESS;
}

void 
Helper::run()
{
  auto robot_pose = this->getRobotObject()->getPose();
  auto goal_pose = this->getGoalObject()->getPose();
  nav_msgs::msg::Path g_path;
  double x_step = (goal_pose.pose.position.x - robot_pose.pose.position.x) / 20.0;
  double y_step = (goal_pose.pose.position.y - robot_pose.pose.position.y) / 20.0;
  double robot_theta = tf2::getYaw(robot_pose.pose.orientation);
  double goal_theta = tf2::getYaw(goal_pose.pose.orientation);
  double theta_step = (goal_theta - robot_theta) / 20.0;
  g_path.header = robot_pose.header;
  for (size_t i = 0; i < 20; i++)
  {
    g_path.poses.push_back(robot_pose);
    robot_pose.pose.position.x += x_step;
    robot_pose.pose.position.y += y_step;
    tf2::Quaternion q;
    q.setEuler(robot_theta, 0.0, 0.0);
    robot_pose.pose.orientation = tf2::toMsg(q);
    robot_theta += theta_step;
  }
  g_path.poses.push_back(goal_pose);
  
  this->controller_->setPlan(g_path);
  try
  {
    auto vel = this->controller_->computeVelocityCommands(
      this->getRobotObject()->getPose(),
      this->getRobotObject()->getVelocity(),
      nullptr
    );
    vel.header.stamp = this->now();
    this->vel_pub_->publish(vel);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
}

object::Robot *
Helper::getRobotObject()
{
  return static_cast<object::Robot *>(this->objects_[0].get());
}

object::Goal * 
Helper::getGoalObject()
{
  return static_cast<object::Goal *>(this->objects_[1].get());
}

void
Helper::updateObjects()
{
  nav2_costmap_2d::Costmap2D * costmap = this->costmap_->getCostmap();
  costmap->resetMapToValue(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
  this->test_layer_->resetObstacle();
  for (auto &obj : this->objects_)
  {
    if(obj->getType() != object::Object::UNIQUE_MOVABLE){
      this->test_layer_->addObstacle(obj);
    }
    obj->updateTF();
  }  
}


} // namespace nav2_config_helper
