#ifndef YGZ_FEATURE_H
#define YGZ_FEATURE_H

#include "ygz/NumTypes.h"
#include "ygz/Settings.h"

using namespace std;

namespace ygz {

    // forward declare
    struct MapPoint;

    // Feature 是指一个图像点，参数化中有它的invDepth。没有关联地图点时，mpPoint为空
    struct Feature {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Vector2f mPixel = Vector2f(0, 0);        // the pixel position
        shared_ptr<MapPoint> mpPoint = nullptr;  // the corresponding map point, nullptr if not associated
        float mfInvDepth = -1;                   // inverse depth, invalid if less than zero.

        // data used in ORB
        float mScore = 0;                        // score, maybe harris or other things
        float mAngle = 0;                        // angle of oriented FAST
        size_t mLevel = 0;                       // the pyramid level
        uchar mDesc[32] = {0};                   // 256 bits of ORB feature (32x8), ignored if using LK flow

        // flags
        bool mbOutlier = false;                  // true if it is an outlier
    };
}


#endif
