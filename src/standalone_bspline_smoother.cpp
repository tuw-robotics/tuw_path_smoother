#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "tuw_path_smoother/bspline_smoother_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<BSplineSmootherNode> node = std::make_shared<BSplineSmootherNode>("bspline_smoother");
  exe.add_node(node);
  exe.spin();

  rclcpp::shutdown();

  return 0;
}