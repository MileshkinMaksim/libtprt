#pragma once

#include "tprt/Core/Types.hpp"
#include "tprt/Kinematics/Christoffel.hpp"
#include <libInterpolate/Interpolators/_1D/CubicSplineInterpolator.hpp>
#include <vector>

namespace tprt {

class VTISlownessSpline {
public:
    VTISlownessSpline() = default;

    // Построение 1D сплайна групповой медленности S_g(theta_g)
    void build(const Tensor4D& A, WaveType wave_type, int num_samples = 91);

    // Получение медленности S_g (с/м) для направления лучевого вектора
    double getSlowness(const Vector3D& ray_direction) const;

    // Вычисление времени пробега на отрезке между 3D точками
    double getSegmentTravelTime(const Point3D& p1, const Point3D& p2) const;

private:
    _1D::CubicSplineInterpolator<double> spline_;
    bool is_initialized_ = false;
};

} // namespace tprt