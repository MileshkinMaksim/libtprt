#include "tprt/Kinematics/VTISlownessSpline.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tprt {

VTISlownessSpline::VTISlownessSpline()
    : spline_(std::make_shared<_1D::CubicSplineInterpolator<double>>()) {}

void VTISlownessSpline::build(const Tensor4D& A, WaveType wave_type, int num_samples) {
    struct Sample {
        double angle;
        double slowness;
    };

    std::vector<Sample> samples;
    samples.reserve(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        double psi = (M_PI_2 * i) / (num_samples - 1);
        Vector3D n(std::sin(psi), 0.0, std::cos(psi));

        ModeKinematics mode = ChristoffelSolver::solveForMode(A, n, wave_type);
        double v_g_norm = mode.group_velocity.norm();
        if (v_g_norm < 1e-8) {
            throw std::runtime_error("Нулевая групповая скорость при построении сплайна медленности.");
        }

        double theta_g = std::atan2(std::abs(mode.group_velocity.x()), std::abs(mode.group_velocity.z()));
        double s_group = 1.0 / v_g_norm;

        samples.push_back({theta_g, s_group});
    }

    // Сортировка точек по лучевому углу для корректной работы CubicSplineInterpolator
    std::sort(samples.begin(), samples.end(), [](const Sample& a, const Sample& b) {
        return a.angle < b.angle;
    });

    std::vector<double> group_angles;
    std::vector<double> slownesses;
    group_angles.reserve(samples.size());
    slownesses.reserve(samples.size());

    for (const auto& s : samples) {
        if (!group_angles.empty() && std::abs(s.angle - group_angles.back()) < 1e-10) {
            continue; // Пропуск совпадающих углов
        }
        group_angles.push_back(s.angle);
        slownesses.push_back(s.slowness);
    }

    spline_->setData(group_angles, slownesses);
    is_initialized_ = true;
}

double VTISlownessSpline::getSlowness(const Vector3D& ray_direction) const {
    if (!is_initialized_) {
        throw std::runtime_error("Сплайн медленности VTI не инициализирован.");
    }

    Vector3D d = ray_direction.normalized();
    double theta_g = std::acos(std::clamp(std::abs(d.z()), 0.0, 1.0));
    return (*spline_)(theta_g);
}

double VTISlownessSpline::getSegmentTravelTime(const Point3D& p1, const Point3D& p2) const {
    Vector3D segment = p2 - p1;
    double length = segment.norm();
    if (length < 1e-12) {
        return 0.0;
    }
    return length * getSlowness(segment);
}

} // namespace tprt