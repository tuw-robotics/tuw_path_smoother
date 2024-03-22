#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

#include "tuw_path_smoother/bspline_smoother.hpp"

class BSplineSmootherNode : public rclcpp::Node
{
public:
    BSplineSmootherNode(const std::string &node_name);

private:
    BSplineSmoother smoother;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;

    void callback_path(const nav_msgs::msg::Path::SharedPtr msg);
};
