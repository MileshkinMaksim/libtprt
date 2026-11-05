#include "tprt/Core/ElasticTensor.hpp"
#include <cmath>
#include <stdexcept>

namespace tprt {

Matrix6d ElasticTensor::isoToVoigt(double vp, double vs, double rho) {
    Matrix6d C = Matrix6d::Zero();
    double c11 = rho * vp * vp;
    double c44 = rho * vs * vs;
    double c12 = c11 - 2.0 * c44;

    C(0, 0) = C(1, 1) = C(2, 2) = c11;
    C(3, 3) = C(4, 4) = C(5, 5) = c44;
    C(0, 1) = C(1, 0) = C(0, 2) = C(2, 0) = C(1, 2) = C(2, 1) = c12;

    return C;
}

Matrix6d ElasticTensor::thomsenToVoigt(const ThomsenParams& p) {
    Matrix6d C = Matrix6d::Zero();

    double c33 = p.rho * p.vp0 * p.vp0;
    double c44 = p.rho * p.vs0 * p.vs0;
    double c11 = (2.0 * p.epsilon + 1.0) * c33;
    double c66 = (2.0 * p.gamma + 1.0) * c44;

    double term = 2.0 * p.delta * c33 * (c33 - c44) + (c33 - c44) * (c33 - c44);
    if (term < 0.0) {
        throw std::invalid_argument("Некорректные параметры Томсена: отрицательное подкоренное выражение для C13.");
    }
    double c13 = std::sqrt(term) - c44;
    double c12 = c11 - 2.0 * c66;

    C(0, 0) = c11; C(1, 1) = c11; C(2, 2) = c33;
    C(3, 3) = c44; C(4, 4) = c44; C(5, 5) = c66;

    C(0, 1) = c12; C(1, 0) = c12;
    C(0, 2) = c13; C(2, 0) = c13;
    C(1, 2) = c13; C(2, 1) = c13;

    return C;
}

Tensor4D ElasticTensor::voigtToTensor4D(const Matrix6d& C_voigt, double rho) {
    Tensor4D A{};

    static const int voigt_map[3][3] = {
        {0, 5, 4},
        {5, 1, 3},
        {4, 3, 2}
    };

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            int I = voigt_map[i][j];
            for (int k = 0; k < 3; ++k) {
                for (int l = 0; l < 3; ++l) {
                    int J = voigt_map[k][l];
                    A[i][j][k][l] = C_voigt(I, J) / rho;
                }
            }
        }
    }

    return A;
}

} // namespace tprt