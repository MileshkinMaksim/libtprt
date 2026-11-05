#include "tprt/IO/JsonParser.hpp"
#include "tprt/Geometry/FlatHorizon.hpp"
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <fstream>
#include <stdexcept>

namespace tprt {

VelocityModel JsonParser::parseVelocityModel(const std::string& filepath) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        throw std::runtime_error("Не удалось открыть файл скоростной модели: " + filepath);
    }

    rapidjson::IStreamWrapper isw(ifs);
    rapidjson::Document doc;
    doc.ParseStream(isw);

    if (doc.HasParseError() || !doc.IsObject() || !doc.HasMember("layers")) {
        throw std::runtime_error("Некорректная структура JSON в файле: " + filepath);
    }

    VelocityModel model;
    const auto& layers = doc["layers"].GetArray();

    for (const auto& layer_val : layers) {
        double z_top = layer_val["z_top"].GetDouble();
        double rho = layer_val.HasMember("rho") ? layer_val["rho"].GetDouble() : 2000.0;

        ThomsenParams p{};
        p.rho = rho;

        if (layer_val.HasMember("thomsen_parameters")) {
            const auto& tp = layer_val["thomsen_parameters"];
            p.vp0 = tp["vp0"].GetDouble();
            p.vs0 = tp["vs0"].GetDouble();
            p.epsilon = tp["epsilon"].GetDouble();
            p.delta = tp["delta"].GetDouble();
            p.gamma = tp["gamma"].GetDouble();
        }

        auto top_horizon = std::make_shared<FlatHorizon>(z_top);
        MediumType m_type = (p.epsilon == 0.0 && p.delta == 0.0 && p.gamma == 0.0) 
                            ? MediumType::ISOTROPIC 
                            : MediumType::VTI;

        model.addLayer(Layer(top_horizon, m_type, p));
    }

    return model;
}

std::vector<ReceiverData> JsonParser::parseReceivers(const std::string& filepath) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        throw std::runtime_error("Не удалось открыть файл приемников: " + filepath);
    }

    rapidjson::IStreamWrapper isw(ifs);
    rapidjson::Document doc;
    doc.ParseStream(isw);

    if (doc.HasParseError() || !doc.IsArray()) {
        throw std::runtime_error("Некорректная структура JSON в файле приемников: " + filepath);
    }

    std::vector<ReceiverData> receivers;
    receivers.reserve(doc.Size());

    for (const auto& item : doc.GetArray()) {
        int rec_num = item["receiver_number"].GetInt();
        const auto& coords = item["coordinates"];
        double x = coords["x"].GetDouble();
        double y = coords["y"].GetDouble();
        double z = coords["z"].GetDouble();

        receivers.push_back({rec_num, Point3D(x, y, z)});
    }

    return receivers;
}

std::vector<ReferenceTimeData> JsonParser::parseReferenceTimes(const std::string& filepath) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        throw std::runtime_error("Не удалось открыть файл эталонных времен: " + filepath);
    }

    rapidjson::IStreamWrapper isw(ifs);
    rapidjson::Document doc;
    doc.ParseStream(isw);

    if (doc.HasParseError() || !doc.IsArray()) {
        throw std::runtime_error("Некорректная структура JSON в файле эталонных времен: " + filepath);
    }

    std::vector<ReferenceTimeData> ref_times;
    ref_times.reserve(doc.Size());

    for (const auto& item : doc.GetArray()) {
        int rec_num = item["receiver_number"].GetInt();
        const auto& times_obj = item["times"];

        ReferenceTimeData data;
        data.receiver_id = rec_num;
        if (times_obj.HasMember("P")) data.times[WaveType::qP] = times_obj["P"].GetDouble();
        if (times_obj.HasMember("S1")) data.times[WaveType::qS1] = times_obj["S1"].GetDouble();
        if (times_obj.HasMember("S2")) data.times[WaveType::qS2] = times_obj["S2"].GetDouble();

        ref_times.push_back(data);
    }

    return ref_times;
}

} // namespace tprt