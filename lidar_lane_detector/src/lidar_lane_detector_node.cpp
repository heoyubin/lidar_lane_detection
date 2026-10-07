#include "rclcpp/rclcpp.hpp"
#include "lidar_lane_detector/lidar_lane_detector.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LaneDetector>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
