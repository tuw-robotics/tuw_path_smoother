#include "tuw_path_smoother/spline.hpp"

Spline::Spline(const Eigen::Spline2d sp)
    : eigen_spline(sp)
{
    const double spline_length = this->d(1);
    this->increase_cache(spline_length);
}

Eigen::Vector2d Spline::get(const long double d)
{
    if (d > this->cache_limit)
    {
        this->increase_cache(d);
    }

    return this->eigen_spline(this->t(d));
}

Eigen::Vector2d Spline::raw(long double t)
{
    return this->eigen_spline(t);
}

Eigen::Vector2d Spline::get_d(const long double d)
{
    if (d > this->cache_limit)
    {
        this->increase_cache(d);
    }

    return this->eigen_spline.derivatives(this->t(d), 1).col(1);
}

Eigen::Vector2d Spline::get_dd(const long double d)
{
    if (d > this->cache_limit)
    {
        this->increase_cache(d);
    }

    return this->eigen_spline.derivatives(this->t(d), 2).col(2);
}

Eigen::Matrix2Xd Spline::getn(const long double d_max)
{
    if (d_max > this->cache_limit)
    {
        this->increase_cache(d_max);
    }

    const int max_i = Spline::get_cache_index(d_max);
    Eigen::Matrix2Xd result(2, max_i + 1);

    const double dd = 1 / pow(10, CACHE_DIGITS);

    for (int i = 0; i <= max_i; i++)
    {
        result.col(i) = this->get(i * dd);
    }

    return result;
}

int Spline::get_cache_index(const long double d)
{
    return (int)lroundl(d * pow(10, CACHE_DIGITS));
}

double Spline::t(const long double d)
{
    if (d > this->cache_limit)
    {
        this->increase_cache(d);
    }

    const int i = this->get_cache_index(d);

    return this->cache(i);
}

double Spline::d(const long double limit)
{
    // TODO: Cleanup this cache for d(1)
    if (limit == 1 && this->d1 != -1)
    {
        return this->d1;
    }

    const double dt = 1 / pow(10, CACHE_DIGITS + 2);

    double d_curr = 0;
    double t_curr = 0;

    while (t_curr < limit)
    {
        const double t_prev = t_curr;
        t_curr += dt;

        Eigen::Vector2d p = this->eigen_spline(t_curr);
        Eigen::Vector2d p_prev = this->eigen_spline(t_prev);

        d_curr += (p - p_prev).norm();
    }

    if (limit == 1)
    {
        this->d1 = d_curr;
    }

    return d_curr;
}

void Spline::increase_cache(const long double limit)
{
    const int new_size = get_cache_index(limit) + 2;

    Eigen::VectorXd new_cache(new_size);
    new_cache(0) = 0;

    const double dt = 1 / pow(10, CACHE_DIGITS + 1);
    const double dd = 1 / pow(10, CACHE_DIGITS);

    double d_target = 0;
    double d_curr = 0;
    double t_curr = 0;

    while (d_target < limit)
    {
        d_target += dd;

        while (d_curr < d_target)
        {
            const double t_prev = t_curr;
            t_curr += dt;

            Eigen::Vector2d p = this->eigen_spline(t_curr);
            Eigen::Vector2d p_prev = this->eigen_spline(t_prev);
            d_curr += (p - p_prev).norm();
        }

        const int i = get_cache_index(d_target);
        new_cache(i) = t_curr;
    }

    // Update cache.
    this->cache = new_cache;
    this->cache_limit = limit;
}

double Spline::inverse(Eigen::Vector2d pos) {
    const double step_size = 0.01;

    double t_max = 0.0;
    const Eigen::Vector2d p0 = this->eigen_spline(0);
    double distance = (pos - p0).norm();

    for (double t = 0.0; t <= 1.0; t += step_size) {
        const Eigen::Vector2d p = this->eigen_spline(t);
        const double d = (pos - p).norm();

        if (d < distance) {
            distance = d;
            t_max = t;
        }
    }

    return t_max;
}
