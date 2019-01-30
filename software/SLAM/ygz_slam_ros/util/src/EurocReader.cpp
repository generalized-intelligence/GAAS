#include "ygz/EurocReader.h"

#include <iomanip>


using namespace std;

namespace ygz {

    bool LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                    vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps) {
        ifstream fTimes;
        fTimes.open(strPathTimes.c_str());
        if (!fTimes) {
            LOG(ERROR) << "cannot find timestamp file: " << strPathTimes << endl;
            return false;
        }
        vTimeStamps.reserve(5000);
        vstrImageLeft.reserve(5000);
        vstrImageRight.reserve(5000);

        while (!fTimes.eof()) {
            string s;
            getline(fTimes, s);
            if (!s.empty()) {
                stringstream ss;
                ss << s;
                vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
                vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
                double t;
                ss >> t;
                vTimeStamps.push_back(t / 1e9);
            }
        }
        fTimes.close();

        if (strPathLeft.empty()) {
            LOG(ERROR) << "No images in left folder!" << endl;
            return false;
        }

        if (strPathRight.empty()) {
            LOG(ERROR) << "No images in right folder!" << endl;
            return false;
        }
        return true;
    }

    bool LoadImus(const string &strImuPath, VecIMU &vImus) {
        ifstream fImus(strImuPath);

        if (!fImus) {
            LOG(ERROR) << "cannot find IMU file: " << strImuPath << endl;
            return false;
        }

        vImus.reserve(30000);
        //int testcnt = 10;
        while (!fImus.eof()) {
            string s;
            getline(fImus, s);
            if (!s.empty()) {
                char c = s.at(0);

                if (c < '0' || c > '9') // skip the comment
                    continue;

                stringstream ss;
                ss << s;
                double tmpd;
                int cnt = 0;
                double data[10];    // timestamp, wx,wy,wz, ax,ay,az
                while (ss >> tmpd) {
                    data[cnt] = tmpd;
                    cnt++;
                    if (cnt == 7)
                        break;
                    if (ss.peek() == ',' || ss.peek() == ' ')
                        ss.ignore();
                }
                data[0] *= 1e-9;
                ygz::IMUData imudata(data[1], data[2], data[3],
                                     data[4], data[5], data[6], data[0]);
                vImus.push_back(imudata);
            }
        }
        fImus.close();
        return true;
    }

    bool LoadGroundTruthTraj(const string &trajPath,
                             TrajectoryType &trajectory) {

        ifstream fTraj(trajPath);
        if (!fTraj) {
            LOG(ERROR) << "cannot find trajectory file!" << endl;
            return false;
        }

        while (!fTraj.eof()) {
            string s;
            getline(fTraj, s);
            if (!s.empty()) {
                if (s[0] < '0' || s[0] > '9') // not a number
                    continue;

                stringstream ss;
                ss << s;
                double timestamp = 0;
                ss >> timestamp;
                ss.ignore();

                timestamp *= 1e-9;

                double data[7];
                for (double &d:data) {
                    ss >> d;
                    if (ss.peek() == ',' || ss.peek() == ' ')
                        ss.ignore();
                }

                // x,y,z,qw,qx,qy,qz
                SE3d pose(SO3d(Quaterniond(data[3], data[4], data[5], data[6])),
                          Vector3d(data[0], data[1], data[2]));
                trajectory[timestamp] = pose;
            }
        }

        fTraj.close();

        return true;
    }
}
