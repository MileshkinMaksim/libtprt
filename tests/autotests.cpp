#include <gtest/gtest.h>
#include "tprt/Core/ElasticTensor.hpp"
#include "tprt/Kinematics/Christoffel.hpp"
#include "tprt/Geometry/FlatHorizon.hpp"
#include "tprt/Model/VelocityModel.hpp"
#include "tprt/Solver/TwoPointBendingTracer.hpp"
#include "tprt/IO/JsonParser.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>

using namespace tprt;

static std::string getDataDirectory() {
    std::vector<std::string> candidates = {
        "Autotest_Data",
        "../Autotest_Data",
        "../tests/Autotest_Data",
        "tests/Autotest_Data",
        "../../Autotest_Data"
    };
    for (const auto& path : candidates) {
        std::ifstream f(path + "/receivers.json");
        if (f.good()) return path;
    }
    return "";
}

TEST(ChristoffelTest, WaveModeOrdering) {
    ThomsenParams p{3000.0, 1500.0, 0.2, 0.1, 0.05, 2200.0};
    auto C_voigt = ElasticTensor::thomsenToVoigt(p);
    auto A = ElasticTensor::voigtToTensor4D(C_voigt, p.rho);

    Vector3D n(0.6, 0.0, 0.8);
    auto modes = ChristoffelSolver::solve(A, n);

    EXPECT_EQ(modes[0].wave_type, WaveType::qP);
    EXPECT_EQ(modes[1].wave_type, WaveType::qS1);
    EXPECT_EQ(modes[2].wave_type, WaveType::qS2);
}

TEST(BendingTracerTest, HomogeneousIsotropic) {
    ThomsenParams p{2000.0, 1000.0, 0.0, 0.0, 0.0, 2000.0};
    VelocityModel model;
    model.addLayer(Layer(std::make_shared<FlatHorizon>(0.0), MediumType::ISOTROPIC, p));

    TwoPointBendingTracer tracer(model);
    Point3D source(1000.0, 0.0, 1000.0);
    Point3D receiver(1005.0, 0.0, 0.0);

    RayCode code;
    code.addSegment(0, WaveType::qP);
    RayTrajectory traj = tracer.solve(source, receiver, code);

    double expected_distance = std::sqrt(5.0 * 5.0 + 1000.0 * 1000.0);
    double expected_time = expected_distance / p.vp0;

    EXPECT_NEAR(traj.total_travel_time, expected_time, 1e-6);
}

TEST(DatasetValidationTest, RunAllAutotestData) {
    std::string data_dir = getDataDirectory();
    if (data_dir.empty()) {
        std::cout << "\n[WARNING] Папка Autotest_Data не найдена. Пропуск тестов.\n";
        GTEST_SKIP();
    }

    Point3D source(1000.0, 0.0, 1000.0);
    auto receivers = JsonParser::parseReceivers(data_dir + "/receivers.json");

    std::cout << "\n=========================================================================================\n";
    std::cout << "          ЗАПУСК АВТОМАТИЧЕСКОГО ТЕСТИРОВАНИЯ СИМУЛЯЦИИ ЛУЧЕВОГО ТРАССИРОВАНИЯ      \n";
    std::cout << "=========================================================================================\n";

    for (int test_num = 1; test_num <= 5; ++test_num) {
        std::string model_file = data_dir + "/model_config_test_" + std::to_string(test_num) + ".json";
        std::string times_file = data_dir + "/Test_times_" + std::to_string(test_num) + ".json";

        VelocityModel model = JsonParser::parseVelocityModel(model_file);
        auto ref_times = JsonParser::parseReferenceTimes(times_file);
        TwoPointBendingTracer tracer(model);

        std::cout << "---> Прогон Теста #" << test_num << " (model_config_test_" << test_num << ".json)\n";

        for (auto wave_type : {WaveType::qP, WaveType::qS1, WaveType::qS2}) {
            std::string wave_name = (wave_type == WaveType::qP) ? "qP" : (wave_type == WaveType::qS1 ? "qS1" : "qS2");
            double sum_abs_err_us = 0.0, sum_sq_err_us = 0.0, max_abs_err_us = 0.0, sum_rel_err_pct = 0.0;
            size_t count = 0;

            RayCode code;
            for (size_t l = 0; l < model.getNumLayers(); ++l) {
                code.addSegment(l, wave_type);
            }

            for (size_t i = 0; i < receivers.size(); ++i) {
                const auto& rec = receivers[i];
                const auto& ref = ref_times[i];

                auto it = ref.times.find(wave_type);
                if (it == ref.times.end()) continue;

                double t_ref = it->second;
                RayTrajectory traj = tracer.solve(source, rec.position, code);
                double t_calc = traj.total_travel_time;

                double abs_err_sec = std::abs(t_calc - t_ref);
                double abs_err_us = abs_err_sec * 1e6;
                double rel_err_pct = (abs_err_sec / t_ref) * 100.0;

                sum_abs_err_us += abs_err_us;
                sum_sq_err_us += abs_err_us * abs_err_us;
                max_abs_err_us = std::max(max_abs_err_us, abs_err_us);
                sum_rel_err_pct += rel_err_pct;
                count++;
            }

            if (count > 0) {
                double mae = sum_abs_err_us / count;
                double rmse = std::sqrt(sum_sq_err_us / count);
                double mre = sum_rel_err_pct / count;

                std::cout << "   Волна " << std::left << std::setw(5) << wave_name << " | "
                          << "MAE: " << std::right << std::setw(8) << std::fixed << std::setprecision(3) << mae << " мкс | "
                          << "RMSE: " << std::right << std::setw(9) << std::fixed << std::setprecision(3) << rmse << " мкс | "
                          << "MaxAE: " << std::right << std::setw(9) << std::fixed << std::setprecision(3) << max_abs_err_us << " мкс | "
                          << "MRE: " << std::right << std::setw(7) << std::fixed << std::setprecision(4) << mre << "%\n";

                EXPECT_LT(mre, 0.5); // Порог относительной ошибки < 0.5%
            }
        }
    }
    std::cout << "=========================================================================================\n\n";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}