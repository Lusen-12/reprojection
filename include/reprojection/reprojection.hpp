#pragma once
#include "reprojection/types.hpp"
#include <optional>

namespace reproj {

struct ReprojectionResult {
    Point2D pixel;
};

std::optional<ReprojectionResult> reproject(
    const Point3D& world_point,
    const CameraIntrinsics& K,
    const CameraExtrinsics& ext);

double pixelDistance(const Point2D& a, const Point2D& b);

} // namespace reproj
