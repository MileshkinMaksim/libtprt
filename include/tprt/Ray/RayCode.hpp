#pragma once

#include "tprt/Core/Types.hpp"
#include <vector>
#include <cstddef>

namespace tprt {

struct RaySegmentCode {
    size_t layer_index;
    size_t target_horizon_index;
    WaveType wave_type;
};

class RayCode {
public:
    RayCode() = default;
    explicit RayCode(const std::vector<RaySegmentCode>& segments) : segments_(segments) {}

    void addSegment(size_t layer_index, size_t target_horizon_index, WaveType wave_type) {
        segments_.push_back({layer_index, target_horizon_index, wave_type});
    }

    void addSegment(size_t layer_index, WaveType wave_type) {
        segments_.push_back({layer_index, layer_index + 1, wave_type});
    }

    const std::vector<RaySegmentCode>& getSegments() const { return segments_; }
    
    const RaySegmentCode& getSegment(size_t index) const {
        if (index >= segments_.size()) {
            throw std::out_of_range("Индекс сегмента луча выходит за границы.");
        }
        return segments_[index];
    }

    size_t getTargetHorizonIndex(size_t segment_index) const {
        return getSegment(segment_index).target_horizon_index;
    }
    
    size_t size() const { return segments_.size(); }
    bool empty() const { return segments_.empty(); }

    void clear() { segments_.clear(); }
    
private:
    std::vector<RaySegmentCode> segments_;
};

} // namespace tprt