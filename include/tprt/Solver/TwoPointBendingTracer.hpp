#pragma once

#include "tprt/Model/VelocityModel.hpp"
#include "tprt/Ray/RayCode.hpp"
#include "tprt/Ray/RayTrajectory.hpp"

namespace tprt {

class TwoPointBendingTracer {
public:
    explicit TwoPointBendingTracer(const VelocityModel& model);

    // Вычисление оптимальной траектории луча между источником и приемником
    RayTrajectory solve(const Point3D& source, const Point3D& receiver, const RayCode& code) const;

private:
    const VelocityModel& model_;

    struct OptimizationData {
        const VelocityModel* model;
        const RayCode* code;
        Point3D source;
        Point3D receiver;
    };

    static double objectiveFunction(unsigned n, const double* x, double* grad, void* func_data);
};

} // namespace tprt