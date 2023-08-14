#include <nav2_config_helper/tester.hpp>

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  auto tester = std::make_shared<nav2_config_helper::Tester>();
  tester->configure();
  tester->activate();
  rclcpp::spin(tester->get_node_base_interface());
  tester->cleanup();
  tester->shutdown();
  return 0;
}
