#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "confidence_mapping/Filter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<confidence_mapping::Filter>());
  rclcpp::shutdown();
  return 0;
}
