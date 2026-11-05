#pragma once

#include "tprt/Geometry/Horizon.hpp"

namespace tprt {

class FlatHorizon : public Horizon {
public:
    // Горизонтальная плоскость z = depth_z0
    explicit FlatHorizon(double depth_z0);

    // Плоскость с заданным углом падения и азимутом (в градусах)
    FlatHorizon(double depth_z0, double dip_deg, double azimuth_deg);

    double getDepth(double x, double y) const override;
    Vector3D getNormal(double x, double y) const override;

private:
    double z0_;
    Vector3D normal_;
};

} // namespace tprt