
#include "tuw_path_smoother/bspline_smoother.hpp"

bool BSplineSmoother::smooth(
    nav_msgs::msg::Path &path,
    const rclcpp::Duration &)
{
    // Generate Eigen matrix from path
    Eigen::Matrix<double, 2, Eigen::Dynamic> waypoints(2, path.poses.size());
    for (size_t i = 0; i < path.poses.size(); i++)
    {
        waypoints(0, i) = path.poses[i].pose.position.x;
        waypoints(1, i) = path.poses[i].pose.position.y;
    }

    if (waypoints.cols() < 3)
    {
        return true;
    }

    // Interpolate a spline (without orientation)
    Spline2d s_interpolated = Eigen::SplineFitting<Spline2d>::Interpolate(waypoints, 2);
    Spline s = Spline(s_interpolated);

    // Sample new waypoints between the original waypoints
    const double waypoint_spacing = 0.1;
    const double total_length = s.d(1);

    // Create an array for new points
    const long estimated_num_points = lround(ceil(total_length / waypoint_spacing)) * 2; // TODO: Estimate better
    Eigen::Matrix2Xd new_points = Eigen::Matrix2Xd(2, estimated_num_points);
    Eigen::Matrix2Xd orientations = Eigen::Matrix2Xd(2, estimated_num_points);

    Eigen::Index ni = 0;

    for (long i = 1; i < waypoints.cols(); i++)
    {
        const double t0 = s.inverse(waypoints.col(i - 1));
        const double d0 = s.d(t0);
        const double t1 = s.inverse(waypoints.col(i));
        const double d1 = s.d(t1);

        const double segment_length = d1 - d0;

        const long n_points = std::max(round(segment_length / waypoint_spacing) - 1.0, 0.0);
        const double segment_spacing = segment_length / (n_points + 1);

        new_points.col(ni) = s.get(d0);
        orientations.col(ni) = s.get_d(d0);
        ni++;

        for (int j = 1; j <= n_points; ++j)
        {
            const double dj = d0 + j * segment_spacing;
            new_points.col(ni) = s.get(dj);
            orientations.col(ni) = s.get_d(dj);

            ni++;
        }
    }

    new_points.col(ni) = s.eigen_spline(1.0);
    orientations.col(ni) = s.eigen_spline.derivatives(1.0, 1).col(1);
    ni++;

    path.poses.clear();
    for (int i = 0; i < ni; i++)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = new_points(0, i);
        pose.pose.position.y = new_points(1, i);
        double yaw = atan2(orientations(1, i), orientations(0, i));

        pose.pose.orientation = tf2::toMsg(tf2::Quaternion(
            tf2::Vector3(0, 0, 1), yaw));
        path.poses.push_back(pose);
    }

    return true;
};
