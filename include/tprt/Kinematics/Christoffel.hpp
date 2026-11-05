#pragma once

#include "tprt/Core/Types.hpp"
#include <array>
#include <Eigen/Dense>

namespace tprt {

struct ModeKinematics {
    WaveType wave_type;
    double phase_velocity = 0.0;
    Vector3D polarization = Vector3D::Zero();
    Vector3D group_velocity = Vector3D::Zero();
};

class ChristoffelSolver {
public:
    // Расчет кинематических параметров для всех мод волн (qP=0, qS1=1, qS2=2)
    static std::array<ModeKinematics, 3> solve(const Tensor4D& A, const Vector3D& n);

    // Расчет кинематических параметров для выбранной моды волны
    static ModeKinematics solveForMode(const Tensor4D& A, const Vector3D& n, WaveType wave_type);
};

} // namespace tprt