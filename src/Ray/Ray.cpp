#include "tprt/Ray/Ray.hpp"

namespace tprt {

Ray::Ray(Point3D source, Point3D receiver, RayCode code)
    : source_(std::move(source)),
      receiver_(std::move(receiver)),
      code_(std::move(code)) {}

} // namespace tprt