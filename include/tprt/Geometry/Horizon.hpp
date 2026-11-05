#pragma once

#include "tprt/Core/Types.hpp"

namespace tprt {

class Horizon {
public:
    virtual ~Horizon() = default;

    // Вычисление глубины z(x, y)
    virtual double getDepth(double x, double y) const = 0;

    // Вычисление внешней нормали к поверхности n(x, y)
    virtual Vector3D getNormal(double x, double y) const = 0;
};

} // namespace tprt