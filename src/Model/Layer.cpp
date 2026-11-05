#include "tprt/Model/Layer.hpp"
#include "tprt/Core/ElasticTensor.hpp"

namespace tprt {

Layer::Layer(std::shared_ptr<Horizon> top_horizon,
             MediumType medium_type,
             const ThomsenParams& thomsen_params)
    : top_horizon_(std::move(top_horizon)),
      medium_type_(medium_type),
      thomsen_params_(thomsen_params) {

    Matrix6d C_voigt;
    if (medium_type_ == MediumType::ISOTROPIC) {
        C_voigt = ElasticTensor::isoToVoigt(thomsen_params_.vp0, thomsen_params_.vs0, thomsen_params_.rho);
    } else {
        C_voigt = ElasticTensor::thomsenToVoigt(thomsen_params_);
    }
    A_ = ElasticTensor::voigtToTensor4D(C_voigt, thomsen_params_.rho);

    slowness_splines_[static_cast<int>(WaveType::qP)].build(A_, WaveType::qP);
    slowness_splines_[static_cast<int>(WaveType::qS1)].build(A_, WaveType::qS1);
    slowness_splines_[static_cast<int>(WaveType::qS2)].build(A_, WaveType::qS2);
}

double Layer::getSegmentTravelTime(const Point3D& p1, const Point3D& p2, WaveType wave_type) const {
    int mode_idx = static_cast<int>(wave_type);
    return slowness_splines_[mode_idx].getSegmentTravelTime(p1, p2);
}

} // namespace tprt