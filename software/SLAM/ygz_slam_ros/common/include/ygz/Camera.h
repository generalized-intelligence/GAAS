#ifndef YGZ_CAMERA_H
#define YGZ_CAMERA_H

#include "ygz/NumTypes.h"

namespace ygz {

    // the basic stereo pinhole camera
    struct CameraParam {

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        CameraParam(const float &_fx, const float &_fy, const float &_cx, const float &_cy, const float _bf = 0)
                : fx(_fx), fy(_fy), cx(_cx), cy(_cy), bf(_bf) {
            K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
            fxinv = 1 / fx;
            fyinv = 1 / fy;
            Kinv = K.inverse();
            f = (fx + fy) * 0.5;
            b = bf / f;
        }

        // from image pixel to camera point
        inline Vector3d Img2Cam(const Vector2f &px) {
            return Vector3d(
                    fxinv * (px[0] - cx),
                    fyinv * (px[1] - cy),
                    1
            );
        }

        float fx = 0;
        float fy = 0;
        float fxinv = 0;
        float fyinv = 0;
        float cx = 0;
        float cy = 0;
        float b = 0;    // baseline in stereo
        float f = 0;    // focal length
        float bf = 0;   // baseline*focal

        Matrix3f K = Matrix3f::Identity();     // intrinsics
        Matrix3f Kinv = Matrix3f::Identity();  // inverse K

    };

}

#endif