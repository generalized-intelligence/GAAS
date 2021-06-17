#ifndef  YGZ_LK_FLOW_H
#define  YGZ_LK_FLOW_H

#include "ygz/Settings.h"
#include "ygz/Frame.h"

namespace ygz {

    /**
     * 使用LK光流跟踪ref中的特征点
     * @param ref 参考帧
     * @param current 当前帧
     * @param trackPts 参考帧中每个点在当前帧中的2D坐标，小于零表示追踪失败
     * @param keepNotConverged 是否保留未收敛点的估计值
     * @return 成功追踪点的数量
     */
    int LKFlow(
            const shared_ptr<Frame> ref,
            const shared_ptr<Frame> current,
            VecVector2f &trackPts,
            bool keepNotConverged = true
    );

    /**
     * 对单个点进行LK追踪，给定两个金字塔
     * @param pyramid1
     * @param pyramid2
     * @param pixel1
     * @param pixel2
     * @return
     */
    bool LKFlowSinglePoint(
            const vector<Mat> &pyramid1,
            const vector<Mat> &pyramid2,
            const Vector2f &pixel1,
            Vector2f &pixel2
    );

    /**
     * 一维的光流，用于左右目的匹配（但是对校正要求太高，不太现实）
     * @param frame
     * @return
     */
    int LKFlow1D(
            const shared_ptr<Frame> frame
    );

    /**
     * OpenCV's lk flow
     *
     */
    int LKFlowCV(
            const shared_ptr<Frame> ref,
            const shared_ptr<Frame> current,
            VecVector2f &refPts,
            VecVector2f &trackedPts
    );

    /**
     * 双线性插值
     * @param x
     * @param y
     * @param gray
     * @return
     */
    inline uchar GetBilateralInterpUchar(
            const float &x, const float &y, const Mat &gray) {
        const float xx = x - floor(x);
        const float yy = y - floor(y);
        uchar *data = gray.data + int(y) * gray.step + int(x);
        return uchar(
                (1 - xx) * (1 - yy) * data[0] +
                xx * (1 - yy) * data[1] +
                (1 - xx) * yy * data[gray.step] +
                xx * yy * data[gray.step + 1]
        );
    }

}

#endif
