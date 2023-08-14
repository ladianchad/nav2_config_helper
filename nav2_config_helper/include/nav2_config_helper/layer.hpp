#ifndef NAV2_CONFIG_HELPER__LAYER_HPP_
#define NAV2_CONFIG_HELPER__LAYER_HPP_

#include <vector>

#include <nav2_costmap_2d/layer.hpp>

#include "object.hpp"

namespace nav2_config_helper
{

namespace layer
{

class TestLayer : public nav2_costmap_2d::Layer
{
public:

  RCLCPP_SMART_PTR_DEFINITIONS(TestLayer)

  virtual void reset() override;

  virtual bool isClearable() override;

  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  /**
   * @brief Actually update the underlying costmap, only within the bounds
   *        calculated during UpdateBounds().
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void addObstacle(const object::Object::SharedPtr obj);

  void resetObstacle();

private:
  std::vector<object::FootPrint> obstacle_footprints_;
};


  
} // namespace layer


} // namespace nav2_config_helper


#endif