#include "reprojection/reprojection.hpp"
#include <cmath>

namespace reproj {

std::optional<ReprojectionResult> reproject(
    const Point3D& Pw,
    const CameraIntrinsics& K,
    const CameraExtrinsics& ext)
{
    const auto& R = ext.R;
    const auto& t = ext.t;

    double Xc = R[0][0]*Pw.x + R[0][1]*Pw.y + R[0][2]*Pw.z + t[0];
    double Yc = R[1][0]*Pw.x + R[1][1]*Pw.y + R[1][2]*Pw.z + t[1];
    double Zc = R[2][0]*Pw.x + R[2][1]*Pw.y + R[2][2]*Pw.z + t[2];

    if (Zc <= 1e-9) {
        return std::nullopt;
    }

    double u = K.fx * (Xc / Zc) + K.cx;
    double v = K.fy * (Yc / Zc) + K.cy;

    return ReprojectionResult{ Point2D{u, v} };
}

double pixelDistance(const Point2D& a, const Point2D& b) {
    double du = a.u - b.u;
    double dv = a.v - b.v;
    return std::sqrt(du*du + dv*dv);
}

} // namespace reproj
