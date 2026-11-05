#pragma once

#include "tprt/Core/Types.hpp"
#include "tprt/Model/VelocityModel.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace tprt {

struct ReceiverData {
    int id;
    Point3D position;
};

struct ReferenceTimeData {
    int receiver_id;
    std::unordered_map<WaveType, double> times;
};

class JsonParser {
public:
    // Загрузка геолого-геофизической скоростной модели из model_config_test_X.json
    static VelocityModel parseVelocityModel(const std::string& filepath);

    // Загрузка расстановки приемников из receivers.json
    static std::vector<ReceiverData> parseReceivers(const std::string& filepath);

    // Загрузка эталонных времен пробега из Test_times_X.json
    static std::vector<ReferenceTimeData> parseReferenceTimes(const std::string& filepath);
};

} // namespace tprt