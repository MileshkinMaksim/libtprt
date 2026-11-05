#pragma once

#include "tprt/Geometry/Horizon.hpp"
#include <libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp>
#include <vector>

namespace tprt {

class GridHorizon : public Horizon {
public:
    GridHorizon(const std::vector<double>& x_grid,
                const std::vector<double>& y_grid,
                const std::vector<double>& z_data);

    double getDepth(double x, double y) const override;
    Vector3D getNormal(double x, double y) const override;

private:
    mutable _2D::BicubicInterpolator<double> interpolator_;
    double dx_, dy_;
};

} // namespace tprt