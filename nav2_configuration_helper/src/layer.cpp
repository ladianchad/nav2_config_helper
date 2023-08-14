#include <nav2_configuration_helper/layer.hpp>

namespace nav2_configuration_helper
{

namespace layer
{

void
TestLayer::reset()
{
}

bool
TestLayer::isClearable()
{
  return true;
}

void
TestLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  *min_x = std::numeric_limits<double>().max();
  *min_y = std::numeric_limits<double>().max();
  *max_x = std::numeric_limits<double>().min();
  *max_y = std::numeric_limits<double>().min();
  for (const auto & footprint : this->obstacle_footprints_)
  {
    for (const auto & point : footprint)
    {
      *min_x = std::min(*min_x, point.x);
      *min_y = std::min(*min_y, point.y);
      *max_x = std::max(*max_x, point.x);
      *max_y = std::max(*max_y, point.y);
    }
  }
}

void
TestLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  for (const auto & footprint : this->obstacle_footprints_)
  {
    for (const auto & point : footprint)
    {
      unsigned int mx, my;
      master_grid.worldToMap(point.x, point.y, mx, my);
      master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

void
TestLayer::addObstacle(const object::Object::SharedPtr obj)
{
  this->obstacle_footprints_.push_back(obj->getFootPrint());
}

void
TestLayer::resetObstacle()
{
  this->obstacle_footprints_.clear();
}


} // namespace layer

  
} // namespace nav2_configuration_helper
