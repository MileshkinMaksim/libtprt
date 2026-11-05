#pragma once

#include "tprt/Core/Types.hpp"
#include "tprt/Geometry/Horizon.hpp"
#include "tprt/Kinematics/VTISlownessSpline.hpp"
#include <memory>
#include <array>

namespace tprt {

class Layer {
public:
    Layer(std::shared_ptr<Horizon> top_horizon,
          MediumType medium_type,
          const ThomsenParams& thomsen_params);

    MediumType getMediumType() const { return medium_type_; }
    const ThomsenParams& getThomsenParams() const { return thomsen_params_; }
    const Tensor4D& getElasticTensor() const { return A_; }

    const Horizon& getTopHorizon() const { return *top_horizon_; }
    std::shared_ptr<Horizon> getTopHorizonPtr() const { return top_horizon_; }

    double getSegmentTravelTime(const Point3D& p1, const Point3D& p2, WaveType wave_type) const;

private:
    std::shared_ptr<Horizon> top_horizon_;
    MediumType medium_type_;
    ThomsenParams thomsen_params_;
    Tensor4D A_;
    std::array<VTISlownessSpline, 3> slowness_splines_;
};

} // namespace tprt