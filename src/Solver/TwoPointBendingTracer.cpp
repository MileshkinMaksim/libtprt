#include "tprt/Solver/TwoPointBendingTracer.hpp"
#include <nlopt.h>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace tprt {

TwoPointBendingTracer::TwoPointBendingTracer(const VelocityModel& model)
    : model_(model) {}

double TwoPointBendingTracer::objectiveFunction(unsigned n, const double* x, double* grad, void* func_data) {
    auto* data = static_cast<OptimizationData*>(func_data);
    const auto& segments = data->code->getSegments();
    size_t num_segments = segments.size();
    size_t num_nodes = num_segments - 1;

    std::vector<Point3D> current_nodes;
    current_nodes.reserve(num_segments + 1);
    current_nodes.push_back(data->source);

    for (size_t i = 0; i < num_nodes; ++i) {
        double node_x = x[2 * i];
        double node_y = x[2 * i + 1];

        // Получаем Z целевого горизонта для i-го промежуточного узла напрямую из модели
        size_t target_horizon_idx = data->code->getTargetHorizonIndex(i);
        double node_z = data->model->getHorizon(target_horizon_idx).getDepth(node_x, node_y);

        current_nodes.emplace_back(node_x, node_y, node_z);
    }
    current_nodes.push_back(data->receiver);

    // Расчет суммарного времени пробега
    double total_time = 0.0;
    for (size_t i = 0; i < num_segments; ++i) {
        const auto& seg = segments[i];
        const auto& layer = data->model->getLayer(seg.layer_index);
        total_time += layer.getSegmentTravelTime(current_nodes[i], current_nodes[i + 1], seg.wave_type);
    }

    // Численный расчет градиента (при использовании градиентного оптимизатора L-BFGS)
    if (grad) {
        double eps = 1e-5;
        std::vector<double> x_temp(n);
        for (unsigned j = 0; j < n; ++j) {
            x_temp[j] = x[j];
        }

        for (unsigned j = 0; j < n; ++j) {
            x_temp[j] = x[j] + eps;
            double t_plus = objectiveFunction(n, x_temp.data(), nullptr, func_data);

            x_temp[j] = x[j] - eps;
            double t_minus = objectiveFunction(n, x_temp.data(), nullptr, func_data);

            grad[j] = (t_plus - t_minus) / (2.0 * eps);
            x_temp[j] = x[j];
        }
    }

    return total_time;
}

RayTrajectory TwoPointBendingTracer::solve(const Point3D& source, const Point3D& receiver, const RayCode& code) const {
    if (code.empty()) {
        throw std::invalid_argument("Код луча не может быть пустым.");
    }

    RayTrajectory trajectory;
    size_t num_segments = code.size();

    // Односегментный прямой луч
    if (num_segments == 1) {
        const auto& seg = code.getSegments()[0];
        const auto& layer = model_.getLayer(seg.layer_index);
        double t_seg = layer.getSegmentTravelTime(source, receiver, seg.wave_type);

        trajectory.points = {source, receiver};
        trajectory.segment_travel_times = {t_seg};
        trajectory.total_travel_time = t_seg;
        trajectory.converged = true;
        return trajectory;
    }

    size_t num_nodes = num_segments - 1;
    unsigned n_vars = static_cast<unsigned>(2 * num_nodes);

    // Начальное приближение промежуточных узлов (линейная интерполяция X и Y)
    std::vector<double> x_vars(n_vars);
    for (size_t i = 0; i < num_nodes; ++i) {
        double alpha = static_cast<double>(i + 1) / static_cast<double>(num_segments);
        x_vars[2 * i] = source.x() + alpha * (receiver.x() - source.x());
        x_vars[2 * i + 1] = source.y() + alpha * (receiver.y() - source.y());
    }

    OptimizationData opt_data{&model_, &code, source, receiver};

    // Настройка оптимизатора NLopt (L-BFGS с ограниченной памятью)
    nlopt_opt opt = nlopt_create(NLOPT_LD_LBFGS, n_vars);
    nlopt_set_min_objective(opt, objectiveFunction, &opt_data);
    nlopt_set_xtol_rel(opt, 1e-7);
    nlopt_set_ftol_rel(opt, 1e-8);
    nlopt_set_maxeval(opt, 500);

    double min_time = 0.0;
    int status = nlopt_optimize(opt, x_vars.data(), &min_time);
    nlopt_destroy(opt);

    // Формирование итоговой траектории
    trajectory.points.push_back(source);
    trajectory.segment_travel_times.reserve(num_segments);

    for (size_t i = 0; i < num_nodes; ++i) {
        double nx = x_vars[2 * i];
        double ny = x_vars[2 * i + 1];

        size_t target_horizon_idx = code.getTargetHorizonIndex(i);
        double nz = model_.getHorizon(target_horizon_idx).getDepth(nx, ny);

        trajectory.points.emplace_back(nx, ny, nz);
    }
    trajectory.points.push_back(receiver);

    for (size_t i = 0; i < num_segments; ++i) {
        const auto& seg = code.getSegments()[i];
        const auto& layer = model_.getLayer(seg.layer_index);
        double t_seg = layer.getSegmentTravelTime(trajectory.points[i], trajectory.points[i + 1], seg.wave_type);
        trajectory.segment_travel_times.push_back(t_seg);
    }

    trajectory.total_travel_time = min_time;
    trajectory.converged = (status >= 1 && status <= 4);

    return trajectory;
}

} // namespace tprt