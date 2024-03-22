#pragma once

#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

const int CACHE_DIGITS = 2;

typedef Eigen::Spline<double, 2, Eigen::Dynamic> Spline2d;

/**
 * @brief Arc-Length Re-parametrized Spline.
 *
 */
class Spline
{
public:
    explicit Spline(Eigen::Spline2d sp);

    /**
     * @brief Returns the point on the spline at the given arc length.
     *
     * @param d arc length
     * @return const Eigen::Vector2d x, y
     */
    Eigen::Vector2d get(long double d);

    Eigen::Matrix2Xd getn(long double limit);

    /**
     * @brief First derivative of the spline.
     *
     * @param d arc length
     * @return const Eigen::Vector2d x', y'
     */
    Eigen::Vector2d get_d(long double d);

    /**
     * @brief Second derivative of the spline.
     *
     * @param d arc length
     * @return const Eigen::Vector2d x'', y''
     */
    Eigen::Vector2d get_dd(long double d);

    // Spline Parameter t at arc length d.
    double t(long double d);

    // Arc Length d at spline parameter t.
    double d(long double t);

    // Raw access to the spline at t
    Eigen::Vector2d raw(long double t);

    // Increase the cache to include the limit d.
    void increase_cache(long double d);

    double inverse(Eigen::Vector2d);

    // Spline
    Eigen::Spline2d eigen_spline;

private:
    // Upper limit of the cache in d.
    long double cache_limit = -1;

    // Cache storing t. The index can be calculated using get_cache_index();
    Eigen::VectorXd cache = Eigen::VectorXd(0);

    // Get the index of the cache for the given arc length d.
    static int get_cache_index(long double d);

    double d1 = -1;
};
