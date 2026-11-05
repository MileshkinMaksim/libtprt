#include "tprt/Geometry/GridHorizon.hpp"
#include <stdexcept>

namespace tprt {

GridHorizon::GridHorizon(const std::vector<double>& x_grid,
                         const std::vector<double>& y_grid,
                         const std::vector<double>& z_data) {
    if (x_grid.size() < 2 || y_grid.size() < 2) {
        throw std::invalid_argument("Размер сетки горизонта должен быть не менее 2x2.");
    }
    dx_ = (x_grid.back() - x_grid.front()) / static_cast<double>(x_grid.size() - 1);
    dy_ = (y_grid.back() - y_grid.front()) / static_cast<double>(y_grid.size() - 1);

    interpolator_.setData(x_grid, y_grid, z_data);
}

double GridHorizon::getDepth(double x, double y) const {
    return interpolator_(x, y);
}

Vector3D GridHorizon::getNormal(double x, double y) const {
    double eps_x = dx_ * 0.01;
    double eps_y = dy_ * 0.01;

    double dz_dx = (interpolator_(x + eps_x, y) - interpolator_(x - eps_x, y)) / (2.0 * eps_x);
    double dz_dy = (interpolator_(x, y + eps_y) - interpolator_(x, y - eps_y)) / (2.0 * eps_y);

    Vector3D normal(dz_dx, dz_dy, -1.0);
    return normal.normalized();
}

} // namespace tprt