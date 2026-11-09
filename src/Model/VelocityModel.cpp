#include "tprt/Model/VelocityModel.hpp"
#include <stdexcept>

namespace tprt {

void VelocityModel::addLayer(const Layer& layer) {
    layers_.push_back(layer);
    horizons_.push_back(layer.getTopHorizonPtr());
}

void VelocityModel::addHorizon(std::shared_ptr<Horizon> horizon) {
    if (!horizon) {
        throw std::invalid_argument("Нельзя добавить нулевой указатель (nullptr) на Horizon в модель.");
    }
    horizons_.push_back(std::move(horizon));
}

const Layer& VelocityModel::getLayer(size_t index) const {
    if (index >= layers_.size()) {
        throw std::out_of_range("Индекс слоя (" + std::to_string(index) + 
                                ") выходит за границы модели (всего слоев: " + 
                                std::to_string(layers_.size()) + ").");
    }
    return layers_[index];
}

const Horizon& VelocityModel::getHorizon(size_t index) const {
    if (index >= horizons_.size()) {
        throw std::out_of_range("Индекс горизонта (" + std::to_string(index) + 
                                ") выходит за границы модели (всего горизонтов: " + 
                                std::to_string(horizons_.size()) + ").");
    }
    return *horizons_[index];
}

std::shared_ptr<Horizon> VelocityModel::getHorizonPtr(size_t index) const {
    if (index >= horizons_.size()) {
        throw std::out_of_range("Индекс горизонта (" + std::to_string(index) + 
                                ") выходит за границы модели (всего горизонтов: " + 
                                std::to_string(horizons_.size()) + ").");
    }
    return horizons_[index];
}

int VelocityModel::getLayerIndexAt(const Point3D& point) const {
    if (layers_.empty()) {
        return -1;
    }

    for (int i = static_cast<int>(layers_.size()) - 1; i >= 0; --i) {
        double z_top = layers_[i].getTopHorizon().getDepth(point.x(), point.y());
        if (point.z() >= z_top) {
            return i;
        }
    }
    return 0;
}

} // namespace tprt