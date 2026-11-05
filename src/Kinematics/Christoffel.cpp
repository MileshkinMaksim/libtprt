#include "tprt/Kinematics/Christoffel.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace tprt {

std::array<ModeKinematics, 3> ChristoffelSolver::solve(const Tensor4D& A, const Vector3D& n) {
    Vector3D n_norm = n.normalized();

    // Заполнение матрицы Кристоффеля Gamma_ik = A_ijkl * n_j * n_l
    Eigen::Matrix3d Gamma = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k) {
            for (int j = 0; j < 3; ++j) {
                for (int l = 0; l < 3; ++l) {
                    Gamma(i, k) += A[i][j][k][l] * n_norm[j] * n_norm[l];
                }
            }
        }
    }

    // Решение задачи на собственные значения
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(Gamma);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("ChristoffelSolver: ошибка сходимости алгоритма поиска собственных значений.");
    }

    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();

    // 1. Поиск qP моды (наибольшее собственное значение / фазовая скорость)
    int p_idx = 0;
    if (eigenvalues[1] > eigenvalues[p_idx]) p_idx = 1;
    if (eigenvalues[2] > eigenvalues[p_idx]) p_idx = 2;

    // Индексы двух сдвиговых мод
    std::vector<int> s_indices;
    for (int i = 0; i < 3; ++i) {
        if (i != p_idx) s_indices.push_back(i);
    }

    // 2. Классификация qSV (qS1) и qSH (qS2) по вертикальной компоненте поляризации e_z
    double ez_0 = std::abs(eigenvectors(2, s_indices[0]));
    double ez_1 = std::abs(eigenvectors(2, s_indices[1]));

    int sv_idx = s_indices[0];
    int sh_idx = s_indices[1];

    if (ez_0 < ez_1) {
        sh_idx = s_indices[0]; // Меньший |e_z| -> qSH (qS2)
        sv_idx = s_indices[1]; // Больший |e_z| -> qSV (qS1)
    }

    std::array<ModeKinematics, 3> modes;

    // qP (индекс 0)
    modes[0].wave_type = WaveType::qP;
    modes[0].phase_velocity = std::sqrt(std::max(0.0, eigenvalues[p_idx]));
    modes[0].polarization = eigenvectors.col(p_idx);

    // qS1 -> qSV (индекс 1)
    modes[1].wave_type = WaveType::qS1;
    modes[1].phase_velocity = std::sqrt(std::max(0.0, eigenvalues[sv_idx]));
    modes[1].polarization = eigenvectors.col(sv_idx);

    // qS2 -> qSH (индекс 2)
    modes[2].wave_type = WaveType::qS2;
    modes[2].phase_velocity = std::sqrt(std::max(0.0, eigenvalues[sh_idx]));
    modes[2].polarization = eigenvectors.col(sh_idx);

    // 3. Расчет вектора групповой скорости для всех мод
    for (int m = 0; m < 3; ++m) {
        if (modes[m].phase_velocity > 1e-12) {
            Vector3D g = modes[m].polarization;
            Vector3D v_g = Vector3D::Zero();
            for (int dim = 0; dim < 3; ++dim) {
                double sum = 0.0;
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        for (int k = 0; k < 3; ++k) {
                            sum += A[i][j][k][dim] * g[i] * g[k] * n_norm[j];
                        }
                    }
                }
                v_g[dim] = sum / modes[m].phase_velocity;
            }
            modes[m].group_velocity = v_g;
        } else {
            modes[m].group_velocity = Vector3D::Zero();
        }
    }

    return modes;
}

ModeKinematics ChristoffelSolver::solveForMode(const Tensor4D& A, const Vector3D& n, WaveType wave_type) {
    auto modes = solve(A, n);
    int idx = static_cast<int>(wave_type);
    if (idx >= 0 && idx < 3) {
        return modes[idx];
    }
    return modes[0];
}

} // namespace tprt