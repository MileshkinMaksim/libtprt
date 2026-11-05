#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>
#include <string>

namespace tprt {

// Типы сейсмических волн (мод-индексы: qP=0, qS1=1, qS2=2)
enum class WaveType {
    qP = 0,
    qS1 = 1,
    qS2 = 2
};

// Тип упругой среды
enum class MediumType {
    ISOTROPIC,
    VTI
};

// 3D узлы и точки
using Point3D = Eigen::Vector3d;
using Vector3D = Eigen::Vector3d;

// 4D тензор жесткости A_ijkl (3x3x3x3)
using Tensor4D = std::array<std::array<std::array<std::array<double, 3>, 3>, 3>, 3>;

// Параметры анизотропии Томсена
struct ThomsenParams {
    double vp0 = 0.0;     // Вертикальная скорость P-волны (м/с)
    double vs0 = 0.0;     // Вертикальная скорость S-волны (м/с)
    double epsilon = 0.0; // Параметр Томсена epsilon
    double delta = 0.0;   // Параметр Томсена delta
    double gamma = 0.0;   // Параметр Томсена gamma
    double rho = 2000.0;  // Плотность (кг/м^3)
};

// Результат трассирования луча
struct RayPathNode {
    Point3D position;
    double travel_time_segment;
};

} // namespace tprt