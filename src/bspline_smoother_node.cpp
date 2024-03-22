
#include "tuw_path_smoother/bspline_smoother_node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

BSplineSmootherNode::BSplineSmootherNode(const std::string &node_name) : Node(node_name)
{
    RCLCPP_INFO(this->get_logger(), "BSplineSmoother created");

    this->smoother = BSplineSmoother();
    this->sub_path = create_subscription<nav_msgs::msg::Path>("plan", 10, std::bind(&BSplineSmootherNode::callback_path, this, _1));
    this->pub_path = create_publisher<nav_msgs::msg::Path>("smoothed_path", 10);
}

void BSplineSmootherNode::callback_path(const nav_msgs::msg::Path::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received path");

    this->smoother.smooth(*msg, rclcpp::Duration(1.0s));

    nav_msgs::msg::Path path = *msg;

    this->pub_path->publish(path);
}
