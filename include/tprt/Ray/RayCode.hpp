#pragma once

#include "tprt/Core/Types.hpp"
#include <vector>
#include <cstddef>

namespace tprt {

struct RaySegmentCode {
    size_t layer_index;
    WaveType wave_type;
};

class RayCode {
public:
    RayCode() = default;
    explicit RayCode(const std::vector<RaySegmentCode>& segments) : segments_(segments) {}

    void addSegment(size_t layer_index, WaveType wave_type) {
        segments_.push_back({layer_index, wave_type});
    }

    const std::vector<RaySegmentCode>& getSegments() const { return segments_; }
    size_t size() const { return segments_.size(); }
    bool empty() const { return segments_.empty(); }

private:
    std::vector<RaySegmentCode> segments_;
};

} // namespace tprt