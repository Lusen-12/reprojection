#pragma once
#include <array>

namespace reproj {

struct Point3D {
    double x, y, z;
};

struct Point2D {
    double u, v;
};

struct CameraIntrinsics {
    double fx, fy;
    double cx, cy;
};

struct CameraExtrinsics {
    std::array<std::array<double, 3>, 3> R;
    std::array<double, 3> t;
};

} // namespace reproj
