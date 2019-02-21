#ifndef YGZ_ORB_MATCHER_H
#define YGZ_ORB_MATCHER_H

#include "ygz/Settings.h"
#include "ygz/NumTypes.h"
#include "ygz/Align.h"

#include <set>

namespace ygz {

    struct Frame;
    struct MapPoint;

    struct Match {
        Match(int _index1 = -1, int _index2 = -1, int _dist = -1) : index1(_index1), index2(_index2), dist(_dist) {}

        int index1 = -1;
        int index2 = -1;
        int dist = -1;
    };

    class ORBMatcher {

    public:

        ORBMatcher(float nnratio = 0.6, bool checkOri = true) : mfNNratio(nnratio), mbCheckOrientation(checkOri) {}

        // 两个描述子之间的距离
        static int DescriptorDistance(const uchar *desc1, const uchar *desc2);

        /**
         * 用词袋检测两个帧间的匹配
         * @param frame1
         * @param frame2
         * @param matches
         * @param only3D if only search points associated with 3D map point
         * @return
         */
        int SearchByBoW(shared_ptr<Frame> frame1, shared_ptr<Frame> frame2, std::vector<Match> &matches,
                        bool only3D = false);

        // Brute-force Search
        int SearchBruteForce(shared_ptr<Frame> frame1, shared_ptr<Frame> frame2, std::vector<Match> &matches);

        // 尝试通过投影关系将LastFrame中的特征与CurrentFrame中的相匹配
        /**
         * @param CurrentFrame 当前帧
         * @param LastFrame 上一个帧
         * @param th 窗口大小
         * @return 匹配数量
         */
        int SearchByProjection(shared_ptr<Frame> CurrentFrame, shared_ptr<Frame> LastFrame, const float th);

        // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
        // Used to track the local map (Tracking)
        /**
         * 寻找从局部地图投影到当前帧的匹配,在Tracker0
         * @param vpMapPoints 局部地图，由后端维护
         * @param th 搜索窗口倍率
         * @return 匹配点数量
         */
        int
        SearchByProjection(shared_ptr<Frame> F, const std::set<shared_ptr<MapPoint>> &vpMapPoints, const float th = 3);

        /**
         * Search by direct project, use image alignment to estimate the pixel location instread of feature matching, can be faster
         * @param F
         * @param vpMapPoints
         * @param th
         * @return
         */
        int SearchByDirectProjection(shared_ptr<Frame> F, const std::set<shared_ptr<MapPoint>> &vpMapPoints);

        /**
         * 三角化地图点
         * @param pKF1
         * @param pKF2
         * @param F12
         * @param vMatchedPairs
         * @param bOnlyStereo
         * @return
         */
        // Matching to triangulate new MapPoints. Check Epipolar Constraint.
        int SearchForTriangulation(shared_ptr<Frame> pKF1, shared_ptr<Frame> pKF2, Matrix3d &F12,
                                   std::vector<Match> &vMatchedPairs);

        /**
         * 检查极线约束是否满足
         * @param feat1 第一个特征
         * @param feat2 第二个特征
         * @param F12 Fundamental matrix
         * @return
         */
        bool
        CheckDistEpipolarLine(const shared_ptr<Feature> feat1, const shared_ptr<Feature> feat2, const Matrix3d &F12);

        // 计算一个帧左右的图像匹配
        enum StereoMethod {
            ORB_BASED, OPTIFLOW_BASED, OPTIFLOW_CV
        }; 
	
	
	// 计算双目匹配的方法，分为基于ORB匹配的和基于双目光流的
        void ComputeStereoMatches(shared_ptr<Frame> f, StereoMethod method = ORB_BASED);
        //void ComputeStereoMatches(shared_ptr <Frame>f,StereoMethod method=OPTIFLOW_CV);

        // 计算地图点的最优描述
        void ComputeDistinctiveDescriptors(shared_ptr<MapPoint> mp);

        void ComputeStereoMatchesORB(shared_ptr<Frame> f);

        // only2Dpoints 是否仅对2D特征点计算
        void ComputeStereoMatchesOptiFlow(shared_ptr<Frame> f, bool only2Dpoints = false);

        void ComputeStereoMatchesOptiFlowCV(shared_ptr<Frame> f);

    private:
        // 计算affine wrap矩阵
        void GetWarpAffineMatrix(
                shared_ptr<Frame> ref,
                shared_ptr<Frame> curr,
                const shared_ptr<Feature> px_ref,
                int level,
                const SE3d &TCR,
                Eigen::Matrix2d &ACR
        );

        // perform affine warp
        void WarpAffine(
                const Eigen::Matrix2d &ACR,
                const cv::Mat &img_ref,
                const Vector2d &px_ref,
                const int &level_ref,
                const shared_ptr<Frame> ref,
                const int &search_level,
                const int &half_patch_size,
                uint8_t *patch
        );

        // 计算最好的金字塔层数
        // 选择一个分辨率，使得warp不要太大
        // ORB每层金字塔默认是1.2倍缩放，所以每缩小一层是1.2*1.2=1.44,取倒数为0.694444444444
        inline int GetBestSearchLevel(
                const Eigen::Matrix2d &ACR,
                const int &max_level) {
            int search_level = 0;
            float D = ACR.determinant();
            while (D > 3.0 && search_level < max_level) {
                search_level += 1;
                D *= setting::invLevelSigma2[1];
            }
            return search_level;
        }

        // 双线性插值
        inline uchar GetBilateralInterpUchar(
                const double &x, const double &y, const cv::Mat &gray) {
            const double xx = x - floor(x);
            const double yy = y - floor(y);
            uchar *data = &gray.data[int(y) * gray.step + int(x)];
            return uchar(
                    (1 - xx) * (1 - yy) * data[0] +
                    xx * (1 - yy) * data[1] +
                    (1 - xx) * yy * data[gray.step] +
                    xx * yy * data[gray.step + 1]
            );
        }

        // 一些谜之函数
        void ComputeThreeMaxima(vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3);

        /**
         * 视线角垂直的时候，取大一点的窗
         * @param viewCos
         * @return
         */
        inline float RadiusByViewingCos(const float &viewCos) {
            if (viewCos > 0.998)
                return 2.5;
            else
                return 4.0;
        }


        float mfNNratio;
        bool mbCheckOrientation;
    };

}

#endif
