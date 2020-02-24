#include "MeshGenerator.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>
#include <thread>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/mat.hpp"
#include <opencv2/imgcodecs.hpp>

using namespace cv;
using namespace std;

const int EXTRACT_COUNT_EACH_BLOCK = 100;
float pt_calc_dist(const Point2f& p1,const Point2f& p2)
{
    return pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2);
}

void do_cvPyrLK(InputOutputArray prev,InputOutputArray next,vector<Point2f>& p2d_v_prev,vector<Point2f>& p2d_v_next,vector<unsigned char>& track_success_v,vector<float>& err,bool do_backward_check)
{
    cv::calcOpticalFlowPyrLK(prev,next,p2d_v_prev,p2d_v_next,track_success_v,err,cv::Size(21, 21), 3,
                              cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
    vector<unsigned char> backward_check_output = track_success_v;
    vector<unsigned char> final_output = track_success_v;
    LOG(INFO)<<"        To track 2d pts size:"<<p2d_v_prev.size();

    if(do_backward_check)
    {
        //进行反向光流检查,有一个阈值限制.
        vector<Point2f> p2d_prev_backward;
        vector<float> backward_err;
        cv::calcOpticalFlowPyrLK(next,prev,p2d_v_next,p2d_prev_backward,backward_check_output,backward_err);
        for(int i = 0;i<backward_check_output.size();i++)
        {
            backward_check_output.at(i) = backward_check_output.at(i) && (pt_calc_dist(p2d_v_prev.at(i),p2d_prev_backward.at(i))<1.0);
        }
    }
    vector<unsigned char> fundamental_match_success;
    cv::Mat fundMat = cv::findFundamentalMat(p2d_v_prev,p2d_v_next,fundamental_match_success);

    int goodMatches_count = 0;
    for(int i = 0;i<fundamental_match_success.size();i++)
    {
        if(fundamental_match_success[i]&&track_success_v[i]&&backward_check_output[i])
        {
            final_output[i] = 1;
        }
        else
        {
            final_output[i] = 0;
        }
    }
    track_success_v = final_output;
}


typedef struct
{
    std::vector<KeyPoint> kps;
    //cv::Mat desp;
    std::vector<Point2f> get_p2f_vec()
    {
        std::vector<Point2f> p2f_vec;
        cv::KeyPoint::convert(this->kps,p2f_vec);
        return p2f_vec;
    }
}PointDetected;

shared_ptr<PointDetected> extractCamKeyPoints_splited(cv::Mat& Img)
{
    auto gftt = cv::GFTTDetector::create(EXTRACT_COUNT_EACH_BLOCK,//50,  // maximum number of corners to be returned
                                                   0.01, // quality level
          10);
    cv::Feature2D* pfeat;
    pfeat = gftt;


    const int rows_count = 3;
    const int cols_count = 3;
    const int img_size_v = Img.rows;
    const int img_size_u = Img.cols;
    shared_ptr<PointDetected> pResult(new PointDetected);
    vector<KeyPoint> out_kps;
    cv::Mat out_feats;
    for(int v_ = 0;v_ < rows_count;v_++)
    {
        for(int u_ = 0; u_ < cols_count;u_++)
        {
            auto img_part = Img.colRange(u_*(img_size_u/cols_count),((u_+1)*(img_size_u/cols_count))>(img_size_u-1)?(img_size_u-1):(u_+1)*(img_size_u/cols_count)
                                         ).rowRange(
                                         v_*(img_size_v/rows_count),((v_+1)*(img_size_v/rows_count))>(img_size_v-1)?(img_size_v-1):(v_+1)*(img_size_v/rows_count)
                                        );
            vector<KeyPoint> kps;
            cv::Mat feats;
            cv::Mat mask;
            //LOG(INFO)<<"        input_size:"<<img_part.cols<<","<<img_part.rows<<endl;
            //LOG(INFO)<<"        detect kps."<<endl;
            pfeat->detect(img_part,kps,mask);
            //LOG(INFO)<<"        kps.size:"<<kps.size()<<endl;

            for(auto& kp:kps)
            {
                //LOG(INFO)<<"original_pos_x"<<kp.pt.x<<endl;
                kp.pt.x += u_*(img_size_u/cols_count);
                kp.pt.y += v_*(img_size_v/rows_count);
                //LOG(INFO)<<"transformed_pos_x"<<kp.pt.x<<endl;
                out_kps.push_back(kp);
            }
        }
    }
    LOG(INFO)<<"output_feats.size():"<<out_feats.cols<<","<<out_feats.rows<<";kps.size():"<<out_kps.size()<<endl;
    pResult->kps = out_kps;
    return pResult;
}

inline bool check_stereo_match(Point2f& pl,Point2f& pr)
{
    const float diff_v_max = 4.0;
    if(abs(pr.y-pl.y) < diff_v_max
         && pl.x>pr.x
         //&&  p2.x-p1.x > 10.0 // minimum disparity :2 //TODO:set into config file.20 太大....10试试?
         &&  pl.x-pr.x > 1.0 // minimum disparity :2 //TODO:set into config file.10 有点大.4试试?
      )
    {
        return true;
    }
    return false;
}









#include "Timer.h"

using namespace MeshGen;
int main(int argc,char**argv)
{
    auto pImg1 = make_shared<cv::Mat>();
    auto pImg2 = make_shared<cv::Mat>();
    *pImg1 = cv::imread(argv[1]);
    *pImg2 = cv::imread(argv[2]);

    ScopeTimer t("mesh_gen_pipeline");
    auto img_pair = make_pair(pImg1,pImg2);


    auto pts_detected = extractCamKeyPoints_splited(*pImg1);
    vector<Point2f> p_prev = pts_detected->get_p2f_vec();
    vector<Point2f> p_next;
    vector<uint8_t> success_vec;
    vector<float> track_err,disps;
    do_cvPyrLK(*pImg1,*pImg2,p_prev,p_next,success_vec,track_err,true);
    for(int i = 0;i<p_prev.size();i++)
    {
        if (!(success_vec.at(i)&&check_stereo_match(p_prev.at(i),p_next.at(i) )))
        {
            success_vec.at(i) = 0;
        }
        else
        {
            disps.push_back(p_prev.at(i).x - p_next.at(i).x);
        }
    }

    vector<Point2f> pleftpts;

    extract_sub_vec(p_prev,pleftpts,success_vec);
    t.watch("tracking finished.");
    MeshGenerator mg;
    mg.generateMeshWithImagePair_vKP_disps(img_pair,pleftpts,disps);

    t.watch("mesh generated 100 times.");

    return 0;
}
