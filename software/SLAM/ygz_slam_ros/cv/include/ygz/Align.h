#ifndef YGZ_ALIGN_H_
#define YGZ_ALIGN_H_

#include "ygz/Settings.h"
#include "ygz/NumTypes.h"

// 有关align部分的算法
// This part is moved from rpg_SVO with modification to support ygz

namespace ygz {

    const int align_halfpatch_size = 4;
    const int align_patch_size = 8;
    const int align_patch_area = 64;

    /**
     * @brief align a pixel with reference image patch
     * 二维对齐
     * @param[in] cur_img The current image
     * @param[in] ref_patch_with_boarder the patch with boarder, used to compute the gradient (or FEJ)
     * @param[in] ref_patch the patch in reference frame, by default is 64x64
     * @param[in] n_iter maximum iterations
     * @param[out] cur_px_estimate the estimated position in current image, must have an initial value
     * @return True if successful
     */
    bool Align2D(
            const cv::Mat &cur_img,
            uint8_t *ref_patch_with_border,
            uint8_t *ref_patch,
            const int n_iter,
            Vector2f &cur_px_estimate);

    /**
     * 一维对齐
     * @param cur_img 当前帧图像
     * @param dir 方向
     * @param ref_patch_with_border    带边界的图像块
     * @param ref_patch 不带边界的图像块
     * @param n_iter 迭代次数
     * @param cur_px_estimate 当前点估计
     * @param h_inv
     * @return
     */
    bool Align1D(
            const cv::Mat &cur_img,
            const Eigen::Vector2f &dir,                  // direction in which the patch is allowed to move
            uint8_t *ref_patch_with_border,
            uint8_t *ref_patch,
            const int n_iter,
            Vector2f &cur_px_estimate,
            double &h_inv);
}

#endif