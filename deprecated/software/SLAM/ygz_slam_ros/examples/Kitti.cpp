/**
 * This program test the kitti dataset
 */

#include <iomanip>
#include <opencv2/opencv.hpp>

#include "ygz/System.h"

using namespace std;
using namespace ygz;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

void LoadGroundTruth(
        const string &gtFile, const vector<double> &vTimestamps,
        map<double, SE3d, std::less<double>, Eigen::aligned_allocator<SE3d>> &traj);

int main(int argc, char **argv) {

    if (argc != 2) {
        cerr << endl << "Usage: ./bin/Kitti path_to_settings" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
    if (fsSettings.isOpened() == false) {
        LOG(FATAL) << "Cannot find config file!" << endl;
        return 1;
    }
    string datasetDir = fsSettings["DatasetDir"];
    LoadImages(datasetDir, vstrImageLeft, vstrImageRight, vTimestamps);

    System system(argv[1]);

    // Load ground truth Trajectory
    string gtDir = fsSettings["GroundTruth"];
    if (gtDir.empty() == false) {
        map<double, SE3d, std::less<double>, Eigen::aligned_allocator<SE3d>> traj;
        LoadGroundTruth(gtDir, vTimestamps, traj);
        system.SetGroundTruthTrajectory( traj );
    }

    const int nImages = vstrImageLeft.size();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    LOG(INFO) << "Start processing sequence ..." << endl;
    LOG(INFO) << "Images in the sequence: " << nImages << endl << endl;

    setting::TBC = SE3d();


    // Main loop
    cv::Mat imLeft, imRight;
    for (int ni = 0; ni < nImages; ni++) {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni]);
        imRight = cv::imread(vstrImageRight[ni]);
        double tframe = vTimestamps[ni];

        if (imLeft.empty() || imRight.empty()) {
            LOG(INFO) << "Cannot load image " << ni << endl;
        }

        // Pass the images to the SLAM system
        system.AddStereo(imLeft, imRight, tframe);
    }

    // Stop all threads
    system.Shutdown();

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps) {
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for (int i = 0; i < nTimes; i++) {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

void LoadGroundTruth(const string &gtFile, const vector<double> &timestamps,
                     map<double, SE3d, std::less<double>, Eigen::aligned_allocator<SE3d>> &traj) {
    ifstream fin(gtFile);
    if (fin) {
        int index = 0;
        while (!fin.eof()) {
            Matrix3d R;
            Vector3d t;
            fin >> R(0, 0) >> R(0, 1) >> R(0, 2) >> t(0, 0) >>
                R(1, 0) >> R(1, 1) >> R(1, 2) >> t(1, 0) >>
                R(2, 0) >> R(2, 1) >> R(2, 2) >> t(2, 0);
            traj[timestamps[index++]] = SE3d(R, t);
            if (index >= timestamps.size())
                break;
            if (fin.good() == false)
                break;
        }
    }
}



