#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include "FeatureFrontEndCV.h"


using namespace std;
using namespace chrono;
using mcs::cvMat_T;
int main(int argc,char** argv)
{
    //cv::Mat* pimg_1 = new cv::Mat(cv::imread("data/0_Marmalade-the-Hamster-has-fun-in-his-cage.jpg") );
    cv::Mat* p_original_img_1 = new cv::Mat(cv::imread("data/mono1.png",0) );
    cvMat_T* pimg_1 = new( cvMat_T );
    #if USE_UMAT_AS_TYPE == 1
    p_original_img_1->copyTo(*pimg_1);
    LOG(INFO)<<"using cv::UMat as cvMat_T."<<endl;
    #else
    pimg_1 =  p_original_img_1;//new cv::Mat(cv::imread("data/mono1.png"));
    LOG(INFO)<<"using original cv::Mat as cvMat_T."<<endl;
    #endif
    LOG(INFO)<<"Mat size:"<<pimg_1->cols<<","<<pimg_1->rows<<"."<<endl;
    shared_ptr<cvMat_T> ptr(pimg_1);
    vector<shared_ptr<cvMat_T> > v;
    //v.push_back(ptr);
    for(int i = 0;i<8;i++)
    {
        v.push_back(ptr);
    }
    //test1.
    //shared_ptr<PointWithFeatureT>
    std::chrono::time_point<std::chrono::steady_clock> t_start,t_start2,t_start3,t_1,t_2,t_3;
    t_start = std::chrono::steady_clock::now();
    shared_ptr<mcs::PointWithFeatureT> pPWFT = mcs::extractCamKeyPoints(*pimg_1,mcs::KEYPOINT_METHOD_GFTT,false);
    t_1 = std::chrono::steady_clock::now();
    auto cost = duration_cast<std::chrono::duration<double> >(t_1 - t_start);
    LOG(INFO)<<"Extract 1 time cost:"<<cost.count()*1000<<"ms."<<endl;

    t_start2 = std::chrono::steady_clock::now();
    vector<shared_ptr<mcs::PointWithFeatureT> > vres1 = mcs::extractMultipleCamKeyPoints(v);
    t_2 = std::chrono::steady_clock::now();
    auto cost2 = duration_cast<std::chrono::duration<double> >(t_2 - t_start2);
    LOG(INFO)<<"Extract 2 time cost:"<<cost2.count()*1000<<"ms."<<endl;

    t_start3 = std::chrono::steady_clock::now();
    vector<shared_ptr<mcs::PointWithFeatureT> > vres2 = mcs::extractMultipleCamKeyPointsMultiThread(v);
    t_3 = std::chrono::steady_clock::now();
    auto cost3 = duration_cast<std::chrono::duration<double> >(t_3 - t_start3);
    LOG(INFO)<<"Extract 3 (mutlithread) time cost:"<<cost3.count()*1000<<"ms."<<endl;
 
    LOG(INFO)<<"Extract finished!"<<endl;
    //cv::Mat vis0;
    cv::Mat vis1,vis2,vis3;
    cv::Mat colored;
    cv::cvtColor(*pimg_1,colored,cv::COLOR_GRAY2BGR);
    cv::drawKeypoints (colored,pPWFT->kps, vis1);
    cv::drawKeypoints (colored,vres1[0]->kps,vis2);
    cv::drawKeypoints (colored,vres2[0]->kps,vis3);
    
    LOG(INFO)<<"Visualizing!"<<endl;

    cv::imshow("1",vis1);
    cv::imshow("2",vis2);
    cv::imshow("3",vis3);
    cv::waitKey(0);
    return 0;
}
