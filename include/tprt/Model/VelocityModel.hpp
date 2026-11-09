#pragma once

#include "tprt/Model/Layer.hpp"

namespace tprt {

class VelocityModel {
public:
    VelocityModel() = default;

    void addLayer(const Layer& layer);
    void addHorizon(std::shared_ptr<Horizon> horizon);
    size_t getNumLayers() const { return layers_.size(); }
    size_t getNumHorizons() const { return horizons_.size(); }
    
    const Layer& getLayer(size_t index) const;

    const Horizon& getHorizon(size_t index) const;
    std::shared_ptr<Horizon> getHorizonPtr(size_t index) const;

    int getLayerIndexAt(const Point3D& point) const;

private:
    std::vector<Layer> layers_;
    std::vector<std::shared_ptr<Horizon>> horizons_;
};

} // namespace tprt