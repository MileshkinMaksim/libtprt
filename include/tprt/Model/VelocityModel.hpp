#pragma once

#include "tprt/Model/Layer.hpp"
#include <vector>

namespace tprt {

class VelocityModel {
public:
    VelocityModel() = default;

    void addLayer(const Layer& layer);
    size_t getNumLayers() const { return layers_.size(); }
    const Layer& getLayer(size_t index) const;

    int getLayerIndexAt(const Point3D& point) const;

private:
    std::vector<Layer> layers_;
};

} // namespace tprt