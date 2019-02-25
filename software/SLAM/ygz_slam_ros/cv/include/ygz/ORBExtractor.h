#ifndef YGZ_ORB_EXTRACTOR_H
#define YGZ_ORB_EXTRACTOR_H

#include "ygz/Settings.h"
#include "ygz/NumTypes.h"

#include <opencv2/core/core.hpp>
#include <list>

/**
 * ORB 的特征提取算法，内部进行了四叉树分块，底层用的是OpenCV的FAST
 * 此外我们提供了来自SVO的特征提取算法，用fast库实现
 *
 * 请注意直接法中的特征提取不太在意重复性，而特征点法需要保持特征点的可重复性以便特征匹配
 */

namespace ygz {

    struct Frame;
    struct Feature;

    // 原版ORB-SLAM用的四叉树形式
    class ExtractorNode {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        ExtractorNode() : bNoMore(false) {}

        void DivideNode(
                ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

        std::list<ExtractorNode, Eigen::aligned_allocator<ExtractorNode>>::iterator lit;
        std::vector<shared_ptr<Feature>> vKeys;
        Vector2i UL, UR, BL, BR;
        bool bNoMore;
    };

    class ORBExtractor {
    public:
        // select the keypoint method
        // 使用哪种方法提取特征点
        // 更建议使用单层的特征点
        typedef enum {
            FAST_MULTI_LEVEL,       // 多层的FAST，每层金字塔都会提，容易提的比较多，不好控制数量
            FAST_SINGLE_LEVEL,      // 单层的FAST，只提最高分辨率那层，变网格，速度快但重复不太好
            ORB_SLAM2,              // 来自ORB-SLAM2的方法，基于四叉树，最费时间但效果好
            OPENCV_ORB,             // OpenCV原版ORB，可以控制点的数量，但均匀性不好
            OPENCV_GFTT             // OpenCV's Good Feature to Track
        } KeyPointMethod;

        // Constructor, 指定提取方法
        ORBExtractor(const KeyPointMethod &method);

        // detect features for frame
        void Detect(
                shared_ptr<Frame> frame,
                bool leftEye = true,     // 提左眼还是右眼
                bool computeRotAndDesc = true   // if we need to compute the rotation and the descriptor?
        );

    private:
        // use the original fast lib to compute keypoints, faster than Opencv's implementation
        void ComputeKeyPointsFast(
                std::vector<std::vector<shared_ptr<Feature>>> &allKeypoints,
                const std::vector<cv::Mat> &pyramid);

        // compute keypoints in single pyramid
        void ComputeKeyPointsFastSingleLevel(std::vector<shared_ptr<Feature >> &allKeypoints, const cv::Mat &image);

        // OpenCV's ORB detector
        void ComputeKeyPointsORBOpenCV(std::vector<std::vector<shared_ptr<Feature>>> &allKeypoints,
                                       const std::vector<cv::Mat> &pyramid);

        void ComputeKeyPointsGFTT(std::vector<std::vector<shared_ptr<Feature>>> &allKeypoints,
                                  const cv::Mat &img);

        // Shi-Tomasi 分数，这个分数越高则特征越优先
        inline float ShiTomasiScore(const cv::Mat &img, const int &u, const int &v) const {
            float dXX = 0.0;
            float dYY = 0.0;
            float dXY = 0.0;
            const int halfbox_size = 4;
            const int box_size = 2 * halfbox_size;
            const int box_area = box_size * box_size;
            const int x_min = u - halfbox_size;
            const int x_max = u + halfbox_size;
            const int y_min = v - halfbox_size;
            const int y_max = v + halfbox_size;

            if (x_min < 1 || x_max >= img.cols - 1 || y_min < 1 || y_max >= img.rows - 1)
                return 0.0; // patch is too close to the boundary

            const int stride = img.step.p[0];
            for (int y = y_min; y < y_max; ++y) {
                const uint8_t *ptr_left = img.data + stride * y + x_min - 1;
                const uint8_t *ptr_right = img.data + stride * y + x_min + 1;
                const uint8_t *ptr_top = img.data + stride * (y - 1) + x_min;
                const uint8_t *ptr_bottom = img.data + stride * (y + 1) + x_min;
                for (int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom) {
                    float dx = *ptr_right - *ptr_left;
                    float dy = *ptr_bottom - *ptr_top;
                    dXX += dx * dx;
                    dYY += dy * dy;
                    dXY += dx * dy;
                }
            }

            // Find and return smaller eigenvalue:
            dXX = dXX / (2.0 * box_area);
            dYY = dYY / (2.0 * box_area);
            dXY = dXY / (2.0 * box_area);
            return 0.5 * (dXX + dYY - sqrt((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
        }

        // compute keypoints using orb slam's octree based implementation
        void
        ComputeKeyPointsOctTree(std::vector<std::vector<shared_ptr<Feature >>> &allKeypoints, vector<cv::Mat> &pyramid);

        // 用四叉树分割所有的点
        std::vector<shared_ptr<Feature>> DistributeOctTree(
                const std::vector<shared_ptr<Feature>> &vToDistributeKeys, const int &minX,
                const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);


        // data
        std::vector<int> mnFeaturesPerLevel;
        std::vector<cv::Point> pattern;
        std::vector<int> umax;

        shared_ptr<Frame> mpFrame = nullptr;

        // grid fast
        // 单层时，GrizSize会随特征点数量浮动
        int mnGridSize;
        int mnGridCols;
        int mnGridRows;

        KeyPointMethod mMethod;

        cv::Mat mOccupancy;

        std::vector<bool> mvbGridOccupancy;

        // options
        bool mbComputeRotAndDesc = true;
    };
}

#endif
