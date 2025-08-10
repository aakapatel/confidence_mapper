#include <grid_map_ros/grid_map_ros.hpp>

#include "confidence_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"

namespace confidence_mapping {

PostprocessingPipelineFunctor::PostprocessingPipelineFunctor(
    std::shared_ptr<rclcpp::Node> nodeHandle)
    : nodeHandle_(nodeHandle), filterChain_("grid_map::GridMap"),
      filterChainConfigured_(false) {
  // TODO (magnus) Add logic when setting up failed. What happens actually if it
  // is not configured?
  if (!nodeHandle->has_parameter("filterChainParametersName_")) {
    nodeHandle_->declare_parameter("filterChainParametersName_",
                                   std::string("postprocessor_pipeline"));
  }

  readParameters();

  publisher_ = nodeHandle_->create_publisher<grid_map_msgs::msg::GridMap>(
      outputTopic_, 1);

  // Setup filter chain.
  if (!nodeHandle->has_parameter("filterChainParametersName_") ||
      !filterChain_.configure(filterChainParametersName_,
                              nodeHandle_->get_node_logging_interface(),
                              nodeHandle_->get_node_parameters_interface())) {
    RCLCPP_WARN(nodeHandle_->get_logger(),
                "Could not configure the filter chain. Will publish the raw "
                "confidence map without postprocessing!");
    return;
  }

  filterChainConfigured_ = true;
}

PostprocessingPipelineFunctor::~PostprocessingPipelineFunctor() = default;

void PostprocessingPipelineFunctor::readParameters() {
  nodeHandle_->get_parameter("output_topic", outputTopic_);
  nodeHandle_->get_parameter("postprocessor_pipeline_name",
                             filterChainParametersName_);
}

grid_map::GridMap PostprocessingPipelineFunctor::operator()(GridMap &inputMap) {
  if (not filterChainConfigured_) {
    RCLCPP_WARN_ONCE(nodeHandle_->get_logger(),
                     "No postprocessing pipeline was configured. Forwarding "
                     "the raw confidence map!");
    return inputMap;
  }
  RCLCPP_INFO(nodeHandle_->get_logger(), "performing Post processing");
  grid_map::GridMap outputMap;
  if (not filterChain_.update(inputMap, outputMap)) {
    RCLCPP_ERROR(nodeHandle_->get_logger(),
                 "Could not perform the grid map filter chain! Forwarding the "
                 "raw confidence map!");
    return inputMap;
  }

  return outputMap;
}

void PostprocessingPipelineFunctor::publish(const GridMap &gridMap) const {
  // Publish filtered output grid map.
  std::unique_ptr<grid_map_msgs::msg::GridMap> outputMessage;
  outputMessage = grid_map::GridMapRosConverter::toMessage(gridMap);
  publisher_->publish(std::move(outputMessage));
  RCLCPP_DEBUG(nodeHandle_->get_logger(),
               "Confidence map raw has been published.");
}

bool PostprocessingPipelineFunctor::hasSubscribers() const {
  return publisher_->get_subscription_count() > 0;
}

} // namespace confidence_mapping
