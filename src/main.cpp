#include "reprojection/reprojection.hpp"
#include <iostream>

using namespace reproj;

int main() {
    CameraIntrinsics K{800.0, 800.0, 640.0, 360.0};

    CameraExtrinsics ext{
        {{{1,0,0},{0,1,0},{0,0,1}}},
        {0.0, 0.0, 0.0}
    };

    Point3D Pw1{0.0, 0.0, 5.0};
    auto r1 = reproject(Pw1, K, ext);
    if (r1) {
        std::cout << "P1 pixel = (" << r1->pixel.u
                  << ", " << r1->pixel.v << ")\n";
    }

    Point2D observed{650.0, 370.0};
    if (r1) {
        double err = pixelDistance(r1->pixel, observed);
        std::cout << "reprojection error = " << err << " px\n";
    }

    Point3D Pw2{0.0, 0.0, -5.0};
    auto r2 = reproject(Pw2, K, ext);
    if (!r2) {
        std::cout << "P2 behind camera -> failed (expected)\n";
    }
    std::cout << "分支开发测试完成" << std::endl;
    return 0;
}
