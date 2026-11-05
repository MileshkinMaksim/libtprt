#include "tprt/Geometry/FlatHorizon.hpp"
#include <cmath>

namespace tprt {

constexpr double DEG2RAD = M_PI / 180.0;

FlatHorizon::FlatHorizon(double depth_z0)
    : z0_(depth_z0), normal_(Vector3D(0.0, 0.0, -1.0)) {}

FlatHorizon::FlatHorizon(double depth_z0, double dip_deg, double azimuth_deg)
    : z0_(depth_z0) {
    double dip = dip_deg * DEG2RAD;
    double az = azimuth_deg * DEG2RAD;

    double nx = std::sin(dip) * std::sin(az);
    double ny = std::sin(dip) * std::cos(az);
    double nz = -std::cos(dip);

    normal_ = Vector3D(nx, ny, nz).normalized();
}

double FlatHorizon::getDepth(double x, double y) const {
    if (std::abs(normal_.z()) < 1e-12) {
        return z0_;
    }
    return z0_ - (normal_.x() * x + normal_.y() * y) / normal_.z();
}

Vector3D FlatHorizon::getNormal(double /*x*/, double /*y*/) const {
    return normal_;
}

} // namespace tprt