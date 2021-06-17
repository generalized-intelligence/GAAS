#ifndef  YGZ_EUROC_READER_H
#define  YGZ_EUROC_READER_H

#include "ygz/Settings.h"
#include "ygz/NumTypes.h"
#include "ygz/IMUData.h"
#include <string>

using namespace std;
// 一些用于EuRoC数据集的IO函数

namespace ygz {


    // Load the stereo image data
    // 输入：左眼图像目录，右眼图像目录，时间戳文件
    // 输出：排序后左眼图像文件路径、右眼图像文件路径、时间戳
    bool LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                    vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

    // Load the IMU data
    bool LoadImus(const string &strImuPath, VecIMU &vImus);

    /**
     * Load the ground truth trajectory
     * @param [in] trajPath the path to trajectory, in euroc will be xxx/state_groundtruth_estimate0/data.csv
     * @param [out] the loaded trajectory
     * @return true if succeed
     */
    typedef map<double, SE3d, std::less<double>, Eigen::aligned_allocator<SE3d>> TrajectoryType;

    bool LoadGroundTruthTraj(const string &trajPath,
                             TrajectoryType &trajectory);
}

#endif
