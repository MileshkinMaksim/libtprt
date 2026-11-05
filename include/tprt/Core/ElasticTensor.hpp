#pragma once

#include "tprt/Core/Types.hpp"
#include <Eigen/Dense>

namespace tprt {

using Matrix6d = Eigen::Matrix<double, 6, 6>;

class ElasticTensor {
public:
    // Расчет матрицы Войгта C_ij (6x6) в Па для изотропной среды
    static Matrix6d isoToVoigt(double vp, double vs, double rho);

    // Перевод параметров Томсена в матрицу Войгта C_ij (6x6) в Па для VTI-среды
    static Matrix6d thomsenToVoigt(const ThomsenParams& p);

    // Преобразование матрицы Войгта C_ij в 4D тензор жесткости A_ijkl = C_ijkl / rho (м^2/с^2)
    static Tensor4D voigtToTensor4D(const Matrix6d& C_voigt, double rho);
};

} // namespace tprt