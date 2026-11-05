#include "tprt/Model/VelocityModel.hpp"
#include <stdexcept>

namespace tprt {

void VelocityModel::addLayer(const Layer& layer) {
    layers_.push_back(layer);
}

const Layer& VelocityModel::getLayer(size_t index) const {
    if (index >= layers_.size()) {
        throw std::out_of_range("Индекс слоя выходит за границы модели.");
    }
    return layers_[index];
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