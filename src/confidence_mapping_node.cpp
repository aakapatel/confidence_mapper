
#include <rclcpp/rclcpp.hpp>
#include "confidence_mapping/ConfidenceMapping.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
     
  auto nodeHandle = rclcpp::Node::make_shared("confidence_mapping");

  nodeHandle->declare_parameter("num_callback_threads", 5);
  nodeHandle->declare_parameter("postprocessor_num_threads", 1);

  confidence_mapping::ConfidenceMapping confidenceMap(nodeHandle);  

  // Spin
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), nodeHandle->get_parameter("num_callback_threads").as_int());
  executor.add_node(nodeHandle);
  executor.spin();
  //rclcpp::spin(nodeHandle);
  rclcpp::shutdown();  
  return 0;
}
