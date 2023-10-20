#include "VCU_canopen_slave_node.h"
#include "asb_system_test_node.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto asb_system_test_node = std::make_shared<ASBSystemTestNode>("asb_system_test_node");
  executor.add_node(asb_system_test_node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
