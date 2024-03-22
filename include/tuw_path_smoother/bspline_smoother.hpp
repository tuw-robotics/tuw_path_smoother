#pragma once

#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <nav2_core/smoother.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tuw_path_smoother/spline.hpp"

typedef Eigen::Spline<double, 2, Eigen::Dynamic> Spline2d;

class BSplineSmoother : public nav2_core::Smoother
{
public:
    /**
     * @brief Construct a new BSplineSmoother object
     *
     */
    BSplineSmoother() = default;

    /**
     * @brief Destroy the BSplineSmoother object
     *
     */
    ~BSplineSmoother() override = default;

    void cleanup() override{};

    void activate() override{};

    void deactivate() override{};

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
        std::string name, std::shared_ptr<tf2_ros::Buffer>,
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
        std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override{};

    bool smooth(
        nav_msgs::msg::Path &path,
        const rclcpp::Duration &max_time) override;
};
