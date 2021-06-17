#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include "FeatureFrontEndCV.h"
#include "Timer.h"

using namespace std;
using namespace chrono;
using mcs::cvMat_T;
int main(int argc,char** argv)
{
    //cv::Mat* pimg_1 = new cv::Mat(cv::imread("data/0_Marmalade-the-Hamster-has-fun-in-his-cage.jpg") );
    /*cv::Mat* p_original_img_1 = new cv::Mat(cv::imread("data/mono1.png",0) );
    cvMat_T* pimg_1 = new( cvMat_T );
    #if USE_UMAT_AS_TYPE == 1
    p_original_img_1->copyTo(*pimg_1);
    LOG(INFO)<<"using cv::UMat as cvMat_T."<<endl;
    #else
    pimg_1 =  p_original_img_1;//new cv::Mat(cv::imread("data/mono1.png"));
    LOG(INFO)<<"using original cv::Mat as cvMat_T."<<endl;
    #endif
    */
    cvMat_T* pimg_1 = new cvMat_T (mcs::IMREAD("data/mono1.png") );
    LOG(INFO)<<"Mat size:"<<pimg_1->cols<<","<<pimg_1->rows<<"."<<endl;
    shared_ptr<cvMat_T> ptr(pimg_1);
    vector<shared_ptr<cvMat_T> > v;
    //v.push_back(ptr);
    for(int i = 0;i<8;i++)
    {
        v.push_back(shared_ptr<cvMat_T>(new cv::Mat(ptr->clone()) ) );//为了测试需要deep copy一次,避免出现同一个图像同一个图像块不同线程crop好几次;
    }
    //test1.
    //shared_ptr<PointWithFeatureT>
    //std::chrono::time_point<std::chrono::steady_clock> t_start,t_start2,t_start3,t_1,t_2,t_3;
    //t_start = std::chrono::steady_clock::now();
    ScopeTimer t1("keyFrame generation timer");
    shared_ptr<mcs::PointWithFeatureT> pPWFT = mcs::extractCamKeyPoints(*pimg_1,mcs::KEYPOINT_METHOD_GFTT,false);
    t1.watch("Extract 1 time cost:");

    ScopeTimer t2("single thread 8 imgs extract");
    vector<shared_ptr<mcs::PointWithFeatureT> > vres1 = mcs::extractMultipleCamKeyPoints(v);
    t2.watch("Extract 2 time cost:");

    ScopeTimer t3("multithread 8 img time cost:");
    vector<shared_ptr<mcs::PointWithFeatureT> > vres2 = mcs::extractMultipleCamKeyPointsMultiThread(v);
    t3.watch("Extract 3 (mutlithread) time cost:");
    //LOG(INFO)<<"Extract 3 (mutlithread) time cost:"<<cost3.count()*1000<<"ms."<<endl;
    ScopeTimer t4("single thread 1 img with orb descriptors:");
    mcs::extractCamKeyPoints(*pimg_1,mcs::KEYPOINT_METHOD_GFTT,true);
    t4.watch("with orb descriptor time cost:");

    ScopeTimer t5("multithread 8 img gftt kps and orb descriptors:");
    mcs::extractMultipleCamKeyPointsMultiThread(v,mcs::KEYPOINT_METHOD_GFTT,true);
    t5.watch("multithread with orb feat time cost:");
 
    LOG(INFO)<<"Extract finished!"<<endl;
    //cv::Mat vis0;
    cv::Mat vis1,vis2,vis3;
    cv::Mat colored;
    //cv::cvtColor(*pimg_1,colored,cv::COLOR_GRAY2BGR);
    colored = *pimg_1;
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
