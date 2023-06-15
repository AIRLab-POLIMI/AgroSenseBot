#include "canopen_slave_node.h"
#include "ros2_bridge_node.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto canopen_slave = std::make_shared<ROS2BridgeNode>("canopen_test_slave");
  executor.add_node(canopen_slave->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
