#pragma once

#include "tprt/Core/Types.hpp"
#include "tprt/Ray/RayCode.hpp"
#include "tprt/Ray/RayTrajectory.hpp"

namespace tprt {

class Ray {
public:
    Ray(Point3D source, Point3D receiver, RayCode code);

    const Point3D& getSource() const { return source_; }
    const Point3D& getReceiver() const { return receiver_; }
    const RayCode& getCode() const { return code_; }

    const RayTrajectory& getTrajectory() const { return trajectory_; }
    void setTrajectory(const RayTrajectory& traj) { trajectory_ = traj; }

private:
    Point3D source_;
    Point3D receiver_;
    RayCode code_;
    RayTrajectory trajectory_;
};

} // namespace tprt