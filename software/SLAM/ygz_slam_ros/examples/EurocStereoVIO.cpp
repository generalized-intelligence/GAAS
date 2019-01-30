/**
 * This is the Euroc stereo visual-inertial odometry program
 * Please specify the dataset directory in the config file
*/

#include <opencv2/opencv.hpp>

#include "ygz/System.h"
#include "ygz/EurocReader.h"

using namespace std;
using namespace ygz;

struct GPS_pos
{
  double x;
  double y;
  double z;
  int time_ms;
};
typedef std::vector<GPS_pos> VecGPS;
void loadGPS(const string& filename,VecGPS& positionVec);




int main(int argc, char **argv) {
/*
    if (argc != 2) {
        LOG(INFO) << "Usage: EurocStereoVIO path_to_config" << endl;
        return 1;
    }*/

    FLAGS_logtostderr = true;
    google::InitGoogleLogging(argv[0]);
/*
    string configFile(argv[1]);
    */

    string configFile("/home/gi/ygz_with_gps/ygz_stereo_inertial_master/examples/Myownds_euros.yaml");
    //string configFile("/home/gi/ygz_with_gps/ygz_stereo_inertial_master/examples/EuRoC.yaml");
    //string configFile("/home/gi/ygz_with_gps/ygz_stereo_inertial_master/examples/MyOwnDS_fixed.yaml");

    cv::FileStorage fsSettings(configFile, cv::FileStorage::READ);

    if (fsSettings.isOpened() == false) {
        LOG(FATAL) << "Cannot load the config file from " << argv[1] << endl;
    }

    //Load fake "GPS" input.

    //bool use_gps=true;
    bool use_gps = false;
    VecGPS gps_vec;
    if (use_gps)
    {
       LOG(INFO)<<"Using noised 'GPS' input!"<<endl;
       loadGPS("/home/gi/ygz_with_gps/ygz_stereo_inertial_master/noised_gps_by_standard.csv",gps_vec);
    }
    System system(configFile.c_str());

    // rectification parameters
    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() ||
        D_r.empty() ||
        rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return 1;
    }

    cv::Mat M1l, M2l, M1r, M2r;
    cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l,
                                M2l);
    cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r,
                                M2r);

    string leftFolder = fsSettings["LeftFolder"];
    string rightFolder = fsSettings["RightFolder"];
    string imuFolder = fsSettings["IMUFolder"];
    string timeFolder = fsSettings["TimeFolder"];

    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimeStamp;
    if (LoadImages(leftFolder, rightFolder, timeFolder, vstrImageLeft, vstrImageRight, vTimeStamp) == false)
        return 1;

    VecIMU vimus;
    if (LoadImus(imuFolder, vimus) == false)
        return 1;

    // read TBC
    cv::Mat Rbc, tbc;
    fsSettings["RBC"] >> Rbc;
    fsSettings["TBC"] >> tbc;
    if (!Rbc.empty() && tbc.empty()) {
        Matrix3d Rbc_;
        Vector3d tbc_;
        Rbc_ <<
             Rbc.at<double>(0, 0), Rbc.at<double>(0, 1), Rbc.at<double>(0, 2),
                Rbc.at<double>(1, 0), Rbc.at<double>(1, 1), Rbc.at<double>(1, 2),
                Rbc.at<double>(2, 0), Rbc.at<double>(2, 1), Rbc.at<double>(2, 2);
        tbc_ <<
             tbc.at<double>(0, 0), tbc.at<double>(1, 0), tbc.at<double>(2, 0);

        setting::TBC = SE3d(Rbc_, tbc_);
    }

    size_t imuIndex = 0;
    size_t gpsIndex = 0;
    for (size_t i = 0; i < vstrImageLeft.size(); i++) {
        cv::Mat imLeft, imRight, imLeftRect, imRightRect;

        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[i], CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[i], CV_LOAD_IMAGE_UNCHANGED);

        if (imLeft.empty() || imRight.empty()) {
            LOG(WARNING) << "Cannot load image " << i << endl;
            continue;
        }

        cv::remap(imLeft, imLeftRect, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightRect, M1r, M2r, cv::INTER_LINEAR);

        // and imu
        VecIMU vimu;

        double tframe = vTimeStamp[i];

        while (1) {
            const ygz::IMUData &imudata = vimus[imuIndex];
            if (imudata.mfTimeStamp >= tframe)
                break;
            vimu.push_back(imudata);
            imuIndex++;
        }

        while (use_gps){
            GPS_pos& pos = gps_vec[gpsIndex];
            gpsIndex++;
            if(pos.time_ms >= tframe)
            {
                break;
            }
        }
        if (use_gps)
        {
            system.AddStereoIMU(imLeftRect, imRightRect, tframe, vimu,gps_vec[gpsIndex].x,gps_vec[gpsIndex].y,gps_vec[gpsIndex].z,use_gps);
        }
        else
        {
            system.AddStereoIMU(imLeftRect, imRightRect, tframe, vimu,0,0,0,use_gps);
        }

    }

    return 0;
}
void loadGPS(const string& filename,VecGPS& positionVec)
{
    ifstream f_gps;
    //string strPathTimeFile = strPathToSequence + "/times.txt";
    f_gps.open(filename.c_str());
    if (!f_gps) {
        LOG(ERROR) << "cannot find GPS file: " << filename << endl;
        return;
    }
    string line;
    while (!f_gps.eof())
    {
      GPS_pos p;

      getline(f_gps,line);
      istringstream stream_line(line);
      vector<string> fields;
      string field;
      while(getline(stream_line,field,','))
      {
        getline(f_gps,field,',');
        fields.push_back(field);
      }
      if(fields.size()>=4)
      {
          p.time_ms = atoi(fields[0].c_str());
          p.x = atof(fields[1].c_str());
          p.y = atof(fields[2].c_str());
          p.z = atof(fields[3].c_str());
          positionVec.push_back(p);
      }
    }
}

