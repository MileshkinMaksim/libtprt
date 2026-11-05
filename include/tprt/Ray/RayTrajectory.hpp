#pragma once

#include "tprt/Core/Types.hpp"
#include <vector>

namespace tprt {

struct RayTrajectory {
    std::vector<Point3D> points;
    std::vector<double> segment_travel_times;
    double total_travel_time = 0.0;
    bool converged = false;
};

} // namespace tprt