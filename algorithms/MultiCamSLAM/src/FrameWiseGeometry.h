#ifndef FRAMEWISE_GEOMETRY_H
#define FRAMEWISE_GEOMETRY_H
#include "Frame.h"
#include "FeatureFrontEndCV.h"
#include "Timer.h"
#include "Visualization.h"



namespace mcs
{
    using namespace std;
    //void doFindEssentialMatrix(vector<KeyPoint>& prev,vector<KeyPoint>& next,cvMat& output_essential_mat);

    //void doFindEssentialMatrix(Frame& f_prev,Frame& f_next);
    void doFindFundamentalMatrix();
    void doPnPRansacMultiCam_Using_BA(Frame& f1,Frame& f2,cv::Mat& output_r_mat,cv::Mat& output_t_vec,bool& output_optimize_success,vector<float>& residual_of_2dkps);
    void trackAndDoSolvePnPRansacMultiCam(Frame& f1,Frame& f2,cv::Mat& output_r_mat,cv::Mat& output_t_vec,
                                bool& output_frame_pnp_ransac_success,vector<bool>& output_cam_match_success,int* ptotal_tracked_points_count = nullptr)
    {
        int total_success_tracked_point_count = 0;
        ScopeTimer t1("trackAndDoSolvePnPRansacMultiCam() timer");
        bool isStereoMode = f1.frame_type==FRAME_TYPE_STEREO;
        if(isStereoMode)
        {
            LOG(INFO)<<"in doSolvePnPRansacMultiCam stage1,f1.cam_info_vec.size:"<<endl;
            LOG(INFO)<<f1.get_stereo_cam_info().size()<<endl;
            output_cam_match_success.resize(f1.get_stereo_cam_info().size());
        }
        int success_count = 0;//TODO:加入一个策略,完成多摄像头之间的筛选.
        //doSolvePnPRansac
        LOG(INFO)<<"in doSolvePnPRansacMultiCam stage2"<<endl;
        auto imgs_prev = f1.getMainImages();
        auto imgs_next = f2.getMainImages();
        LOG(INFO)<<"in doSolvePnPRansacMultiCam stage3"<<endl;

        auto p3ds_vv = f1.p3d_vv;
        auto pts_vv = f2.p2d_vv;
        //pts_vv = OptFlowForFrameWiseTracking(*pframe2);//track frame2 获取对应位置的点...
        LOG(INFO)<<"in doSolvePnPRansacMultiCam stage4"<<endl;
        for(int i = 0;i < imgs_prev.size();i++)
        {
            LOG(INFO)<<"frame1 p3ds_vv.size():"<<p3ds_vv.size()<<";frame2 p2ds_vv.size():"<<pts_vv.size()<<endl;

            LOG(INFO)<<"in doSolvePnPRansacMultiCam stage4.1"<<endl;
            cv::Mat camMat;
            if(isStereoMode)
            {
                camMat = f2.get_stereo_cam_info()[i].getCamMat();
            }
            else
            {
                camMat = f2.get_cam_info()[i].getCamMat();
            }
            LOG(INFO)<<"in doSolvePnPRansacMultiCam stage4.2"<<endl;
            output_r_mat = cv::Mat();
            output_t_vec = cv::Mat();
            vector<unsigned char> match_success;
            //TODO:multi_thread implementation.


            vector<p3dT> pnp_ransac_input_p3d;
            vector<p2dT> pnp_ransac_input_p2d;
            {//step<1>.track 2d pts.
                vector<p3dT> vp3d_pts = p3ds_vv[i];
                vector<p2dT> vp2d_pts;
                for(int index_p3d=0;index_p3d<vp3d_pts.size();index_p3d++)
                {
                    vp2d_pts.push_back(f1.p2d_vv[i][f1.map3d_to_2d_pt_vec[i][index_p3d] ]);
                }
                vector<cv::KeyPoint> toTrack_2dpts;
                cv::KeyPoint::convert(vp2d_pts,toTrack_2dpts);
                vector<Point2f> original_remaining_2dpts,tracked_nextimg_2dpts;
                vector<unsigned char> v_pt_track_success;
                bool output_track_success;
                map<int,int> tracked_kps_to_original_kps_map_output;
                OptFlowForFrameWiseTracking(*(imgs_prev[i]),*(imgs_next[i]),toTrack_2dpts,original_remaining_2dpts,tracked_nextimg_2dpts,tracked_kps_to_original_kps_map_output,v_pt_track_success,output_track_success);


                for(int index_p3d = 0;index_p3d<vp3d_pts.size();index_p3d++)
                {
                    int p2d_index = tracked_kps_to_original_kps_map_output[index_p3d];
                    if(p2d_index>=0)//a match has been found.
                    {
                        pnp_ransac_input_p2d.push_back(vp2d_pts[p2d_index]);
                        pnp_ransac_input_p3d.push_back(vp3d_pts[p2d_index]);
                    }
                }
            }
            //step<2>.process result.
            LOG(INFO)<<"in doSolvePnPRansacMultiCam stage5: success tracked vp3d_pts.size:"<<pnp_ransac_input_p2d.size()<<",tracked p2d_pts.size:"<<pnp_ransac_input_p2d.size()<<endl;
            LOG(INFO)<<"cam mat rows and cols:"<<camMat.rows<<","<<camMat.cols;
            bool success = solvePnPRansac(pnp_ransac_input_p3d,pnp_ransac_input_p2d,camMat,cv::Mat(),output_r_mat,output_t_vec,false,100,8.0,0.99,match_success,SOLVEPNP_ITERATIVE);
            cv::Mat R_output;
            cv::Rodrigues(output_r_mat,R_output);
            LOG(INFO)<<"in doSolvePnPRansacMultiCam stage6,output R_mat:\n"<<R_output<<"\n output t vec"<<output_t_vec<<endl;
            output_cam_match_success[i] = success;
            if(success)//一组摄像头成功
            {
                int pnpransac_success_count = 0;
                for(int j = 0;j<match_success.size();j++)
                {
                    if(match_success[j])
                    {
                        pnpransac_success_count+=1;
                        total_success_tracked_point_count+=1; //只计算整组成功以后的成功点个数.失败的摄像头组整组都不算了.
                    }
                }
                LOG(INFO)<<"pnp_ransac_match_success!!matched:"<< pnpransac_success_count<<"/"<<pnp_ransac_input_p2d.size()<<endl;
                success_count+=1;
            }
            else
            {
                LOG(WARNING)<<"Cam "<<i<<" has caught into pnp ransac failure!"<<endl;
            }
        }
        LOG(INFO)<<"in doSolvePnPRansacMultiCam stage7"<<endl;
        if(success_count>=3)
        {
            output_frame_pnp_ransac_success = true;
        }
        LOG(INFO)<<"For all cams:total tracked points count:"<<total_success_tracked_point_count;
        if(ptotal_tracked_points_count != nullptr)
        {
            *ptotal_tracked_points_count = total_success_tracked_point_count;
        }
        return;

    }
    void trackLocalFramePoints(Frame& f1,Frame& f2,cv::Mat& output_r_mat,cv::Mat& output_t_vec,
                               bool& output_frame_tracking_success,vector<bool> output_cam_track_success);

    void trackMapPoints(Frame& f1,vector<shared_ptr<MapPoint> > v_pPoints,cv::Mat& output_r_mat, cv::Mat& output_t_vec,
                        bool& output_frame_tracking_success,vector<bool> output_framekp2d_track_success);

    void doTrackLocalMap(shared_ptr<Frame> pFrame);//跟踪局部还在滑窗中的地图点




    void doTrackLaskKF_all2dpts(shared_ptr<Frame> pFrame,shared_ptr<Frame> pKeyFrameReference,int cam_index,
                                //const vector<Point2f>& input_kf_p2d,
                                vector<Point2f>& output_tracked_pts_left,
                                //vector<Point2f>& output_tracked_pts_right,//不需要了.只存disparity,不再保存p2dT数值.
                                vector<float>& disps,//双目追踪失败,disp填入-1.这里的disps是本帧追踪产生的disps,与关键帧无关.
                                vector<char>& output_track_type,//跟踪类型:有KFStereoCurrentMono,KFMonoCurrentStereo,KFStereoCurrentStereo,KFMonoCurrentMono.
                                map<int,int>& map_point2f_to_kf_p2ds,//必然有.
                                map<int,int>& map_point2f_to_kf_p3ds,//没有填写-1;
                                char* p_output_track_success,//调用方必须检查.
                                int* success_tracked_stereo_pts_count,int method = 0)
    {
        //TODO:Implementation of this function!
        ScopeTimer t_dotrackKFall2dpts("    In doTrackLastKF");
        LOG(INFO)<<"in doTrackLastKF_all2dpts."<<endl;
        if(pKeyFrameReference == nullptr)
        {
            LOG(ERROR)<<"reference Frame is nullptr!"<<endl;
            exit(-1);//DEBUG ONLY.
        }
        LOG(INFO)<<"for frame id:"<<pFrame->frame_id<<" calling doTrackLastKF_all2dpts!Reference KFid:"<<pKeyFrameReference->frame_id<<endl;
        const int& i = cam_index;
        *p_output_track_success = false;
        auto p_origin_img = pKeyFrameReference->getMainImages().at(i);
        auto p_left =  pFrame->getMainImages().at(i);
        auto p_right= pFrame->getSecondaryImages().at(i);
        vector<p3dT> vp3d_pts = pKeyFrameReference->p3d_vv.at(i);
        //vector<p2dT>& to_track_vp2d_pts = output_to_track_kf_p2d;//要追踪的参考帧 kps.

        //to_track_vp2d_pts.clear();
        LOG(INFO)<<"    cam index:"<<cam_index<<",p2d_vv[i].size():"<<pKeyFrameReference->p2d_vv.at(i).size()<<endl;

        if(pKeyFrameReference->p2d_vv.at(i).size() == 0)
        {
            return;//失败,返回.
        }
        vector<float> err;
        vector<Point2f> left_tracked_pts;
        vector<unsigned char> left_tracked_success_v;

        vector<Point2f> right_tracked_pts;
        vector<unsigned char> right_track_success_v;
        vector<float> err_right;
        t_dotrackKFall2dpts.watch("Data integrety checked...");
        LOG(INFO)<<"pFrame id:"<<pFrame->frame_id<<" ; pRefKF id:"<<pKeyFrameReference->frame_id<<endl;
        try
        {
            do_cvPyrLK(*p_origin_img,*p_left,pKeyFrameReference->p2d_vv.at(i),left_tracked_pts,left_tracked_success_v,err);//进行追踪.
        }
        catch (Exception e)
        {
            LOG(ERROR)<<"Caught exception in doTrackLastKF_all2dpts()-->do_cvPyrLK for KF and leftImage,error is: "<<e.what()<<endl;
            return;
        }
        try
        {
            do_cvPyrLK(*p_left,*p_right,left_tracked_pts,right_tracked_pts,right_track_success_v,err_right);
        }
        catch (Exception e)
        {
            LOG(ERROR)<<"Caught exception in doTrackLastKF_all2dpts()-->do_cvPyrLK for leftImage and rightImage,error is: "<<e.what()<<endl;
            return;
        }
        t_dotrackKFall2dpts.watch("PyrLK*2 done...");
        //Merge the result of these 2 process.
        int kf_stereo_output_stereo_count = 0;//都是双目追踪.
        int kf_stereo_output_mono_count = 0;//KF双目,当前单目.
        int kf_mono_output_stereo_count = 0;//KF单目,当前双目.
        int kf_mono_output_mono_count = 0;//都是单目.
        //可以根据这个东西算一个评分.

        for(int pt_index = 0;pt_index<left_tracked_pts.size();pt_index++)
        {//检查是否追踪成功.
            if(left_tracked_success_v.at(pt_index)&&right_track_success_v.at(pt_index)&&check_stereo_match(right_tracked_pts.at(pt_index),left_tracked_pts.at(pt_index)))
            {
                int output_id = output_tracked_pts_left.size();
                output_tracked_pts_left.push_back(left_tracked_pts.at(pt_index));
                //output_tracked_pts_right.push_back(right_tracked_pts.at(pt_index));//这个东西不要了.
                map_point2f_to_kf_p2ds[output_id] = pt_index;
                float disp_ = left_tracked_pts.at(pt_index).x - right_tracked_pts.at(pt_index).x;
                disps.push_back(disp_);
                //填充当前帧到参考关键帧p3d的index,无对应p3d填-1;
                if(pKeyFrameReference->map2d_to_3d_pt_vec.at(i).count(pt_index)&&pKeyFrameReference->map2d_to_3d_pt_vec.at(i).at(pt_index)!=-1)
                {
                    map_point2f_to_kf_p3ds[output_id] = pKeyFrameReference->map2d_to_3d_pt_vec.at(i).at(pt_index);
                    kf_stereo_output_stereo_count++;
                    output_track_type.push_back(TRACK_STEREO2STEREO);
                }
                else
                {
                    kf_mono_output_stereo_count++;
                    output_track_type.push_back(TRACK_MONO2STEREO);//关键帧mono,当前帧stereo.
                }
                //TODO:维护TrackType和右侧追踪记录修改为disp.
            }
            else if(left_tracked_success_v.at(pt_index)&&right_track_success_v.at(pt_index)&&check_stereo_v(right_tracked_pts.at(pt_index),left_tracked_pts.at(pt_index)))
            {
                int output_id = output_tracked_pts_left.size();
                output_tracked_pts_left.push_back(left_tracked_pts.at(pt_index));
                map_point2f_to_kf_p2ds[output_id] = pt_index;

                disps.push_back(-1.0);//跟踪失败,disps.push_back(-1.0)
                //左目追踪成功,本帧三角化不成功.
                if(pKeyFrameReference->map2d_to_3d_pt_vec.at(i).count(pt_index)&&pKeyFrameReference->map2d_to_3d_pt_vec.at(i).at(pt_index)!=-1)
                {
                    output_track_type.push_back(TRACK_STEREO2MONO);
                    kf_stereo_output_mono_count++;
                }
                else
                {
                    output_track_type.push_back(TRACK_MONO2MONO);//这种没什么用.
                    kf_mono_output_mono_count++;
                }
            }
        }
        t_dotrackKFall2dpts.watch("output bool values updated...");
        *success_tracked_stereo_pts_count = output_tracked_pts_left.size();
        if(*success_tracked_stereo_pts_count > 15)
        {
            *p_output_track_success = true;
        }
    }
    void doTrackLastKF(shared_ptr<Frame> pFrame,shared_ptr<Frame> pKeyFrameReference,int cam_index,
                                             vector<Point2f>& output_tracked_pts_left,vector<Point2f>& output_tracked_pts_right,
                                             map<int,int>& map_point2f_to_kf_p3ds,vector<p2dT>& output_to_track_kf_p2d,char* p_output_track_success,int* success_tracked_stereo_pts_count,int method = 0)
    {//这个函数要支持多线程.//要能返回平均视差(mean disparity)用于查看关键帧之间创建的逻辑.
        cout<<"in doFrontEndTrackingForOrdinaryFrames():check frame_1,2 integrity."<<endl;
        //pFrame->checkFrameIntegrity_debug();
        //pKeyFrameReference->checkFrameIntegrity_debug();
        //step<1> 追踪左目.
        if(pKeyFrameReference == nullptr)
        {
            cout<<"ERROR:in doFrontEndTrackingForOrdinaryFrames():pKeyFrameReference is nullptr!!!!"<<endl;
        }

        cout<<"in doFrontEndTracking For ordinary frame:stage<1>.current frame id:"<<pFrame->frame_id<<",referring frame id:"<<pKeyFrameReference->frame_id<<endl;
        const int& i = cam_index;
        *p_output_track_success = false;
        auto p_origin_img = pKeyFrameReference->getMainImages().at(i);
        cout<<"in doFrontEndTracking For ordinary frame:stage<2>."<<endl;
        auto p_left = pFrame->getMainImages().at(i);
        cout<<"in doFrontEndTracking For ordinary frame:stage<3>."<<endl;
        auto p_right= pFrame->getSecondaryImages().at(i);
        cout<<"in doFrontEndTracking For ordinary frame:stage<4>."<<endl;
        vector<p3dT> vp3d_pts = pKeyFrameReference->p3d_vv.at(i);
        cout<<"in doFrontEndTracking For ordinary frame:stage<5>."<<endl;
        vector<p2dT>& to_track_vp2d_pts = output_to_track_kf_p2d;//要追踪的参考帧 kps.

        to_track_vp2d_pts.clear();
        cout<<"    cam index:"<<cam_index<<",p2d_vv[i].size():"<<pKeyFrameReference->p2d_vv.at(i).size()<<endl;

        cout<<"map_3d_to_2d_pts_vec.size():"<< pKeyFrameReference->map3d_to_2d_pt_vec.size()<<";acessing index:"<<i<<endl;
        auto map3d_to_2d_pt_vec__ = pKeyFrameReference->map3d_to_2d_pt_vec.at(i);
        cout<<"map_info:size:"<<map3d_to_2d_pt_vec__.size()<<endl;//有时这个map大小为0,引发异常.
        if(map3d_to_2d_pt_vec__.size()==0)
        {
            for(auto& map__ :pKeyFrameReference->map3d_to_2d_pt_vec)
            {
                cout<<"map 3_2 at pKeyFrameRef size:"<<map__.size()<<endl;
            }
        }
        cout<<"     copy of map in stack."<<endl;
        //错误就在这个里面.
        for(int index_p3d=0;index_p3d<vp3d_pts.size();index_p3d++)
        {
            //cout<<"     querying index_p3d:"<<index_p3d<<endl;
            if(map3d_to_2d_pt_vec__.count(index_p3d) == 0)
            {
                cout<<"ERROR: no such p3d."<<endl;
            }
            else
            {
                //cout<<"     p3d_matching p2d_index:"<<map3d_to_2d_pt_vec__.at(index_p3d)<<endl;
            }
            if(pKeyFrameReference-> p2d_vv.at(i).size()<= map3d_to_2d_pt_vec__.at(index_p3d))
            {
                cout<<"ERROR:mapping index overflow!"<<map3d_to_2d_pt_vec__.at(index_p3d)<<endl;
            }
            else
            {
                //cout<<"     pKeyFrameReference->map3d_to_2d_pt_vec.at(i).at(index_p3d):"<<map3d_to_2d_pt_vec__.at(index_p3d)<<endl;

            }
            to_track_vp2d_pts.push_back(pKeyFrameReference->p2d_vv.at(i).at(map3d_to_2d_pt_vec__.at(index_p3d) ));//查找对应关键帧的p2d.只追踪成功三角化的点.
        }
        if(to_track_vp2d_pts.size() == 0)
        {
            cv::imwrite("error_img.jpg",*pKeyFrameReference->getMainImages().at(i));
            cout<<"caught img p2d empty error.error_img.jpg saved!"<<endl;
            *p_output_track_success = false;
            *success_tracked_stereo_pts_count = 0;
            return;
            //throw "to track vp2d empty.error.";//DEBUG ONLY
        }
        //{//DEBUG ONLY!
        //    to_track_vp2d_pts.push_back(pKeyFrameReference->p2d_vv.at(i).at(0));
        //}
        cout<<"in doFrontEndTracking For ordinary frame:stage<6>."<<endl;

        vector<unsigned char> left_track_success;
        vector<Point2f> left_tracked_pts;
        {//call cv::OptFlow for left img.
            cout<<"in doFrontEndTracking For ordinary frame:stage<7>."<<endl;
            vector<float> err;
            try
            {
                cout<<"before cv opt flow pyrlk, count shared_ptr usage:p origin img:"<<p_origin_img.use_count()<<",p_left:"<<p_left.use_count()<<endl;
                auto m_origin = *p_origin_img;
                cout <<"p_origin_img->cols"<<p_origin_img->cols<<"rows:"<<p_origin_img->rows<<endl;
                cout<<"deref p_origin finished!"<<endl;
                auto m_left = *p_left;
                cout<<"pleft_size:"<<p_left->cols<<","<<p_left->rows<<endl;
                cout<<"deref p_left finished!"<<endl;
                cout<<"to track p2f count:"<<to_track_vp2d_pts.size()<<";left_tracked_pts.size:"<<left_tracked_pts.size()<<endl;
                cout<<"img channels:img1:"<<p_origin_img->channels()<<";img2:"<<p_left->channels()<<endl;
                //cout<<"writing img..."<<endl;
                //cv::imwrite("origin.jpg",*p_origin_img);
                //cv::imwrite("left.jpg",*p_left);
                //cv::imwrite("right.jpg",*p_right);
                //cout<<"img saved!"<<endl;
                do_cvPyrLK(*p_origin_img,*p_left,to_track_vp2d_pts,left_tracked_pts,left_track_success,err);

                //cv::calcOpticalFlowPyrLK(*p_origin_img,*p_left,to_track_vp2d_pts,left_tracked_pts,left_track_success,err);//,cv::Size(21, 21), 3,
                                     //cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, , 0.01));
            }
            catch(Exception e)//DEBUG ONLY!
            {
                cout<<"in cv:: opt flow pyr lk error caught! info:"<<e.what()<<endl;
                LOG(ERROR)<<"in cv:: opt flow pyr lk error caught! info:"<<e.what()<<endl;
                throw e;
            }
            cout<<"in doFrontEndTracking For ordinary frame:stage<8>."<<endl;
        }

        //if(method == 0)
        //{
        //    just return the coresponding pts.
        //}
        //else if(method == 1)
        {//双目性质的追踪.
            vector<unsigned char> right_track_success;
            vector<Point2f> right_tracked_pts;
            vector<float> err2;
            cout<<"DEBUG: in doFrontEndTrackingForOrdinaryFrames() before pyrlk: p_left:"<<p_left<<",p_right:"<<p_right<<",left_tracked_pts.size():"<<left_tracked_pts.size()<<",right_tracked_pts.size()"<<right_tracked_pts.size()<<endl;
            //cv::calcOpticalFlowPyrLK(*p_left,*p_right,left_tracked_pts,right_tracked_pts,right_track_success,err2,cv::Size(21,21),3,
            //                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
            do_cvPyrLK(*p_left,*p_right,left_tracked_pts,right_tracked_pts,right_track_success,err2);
            //step<2> 筛选关键帧和左侧能匹配,且左侧和右侧能匹配的.
            cout<<"after calcOptflow pyrlk:"<<endl;
            for(int pt_index = 0;pt_index<left_tracked_pts.size();pt_index++)
            {
                //step<3> 生成最终结果并产生map.
                if(left_track_success.at(pt_index) && right_track_success.at(pt_index)
                            &&check_stereo_match(right_tracked_pts.at(pt_index),left_tracked_pts.at(pt_index) )
                        )
                {//双目成功//TODO:check diff v in left and right.
                    map_point2f_to_kf_p3ds[output_tracked_pts_left.size()] = pt_index;
                    output_tracked_pts_left.push_back(left_tracked_pts.at(pt_index));
                    output_tracked_pts_right.push_back(right_tracked_pts.at(pt_index));
                }
                else if(left_track_success.at(pt_index) && !right_track_success.at(pt_index))
                {//只有单目追踪成功.
                    LOG(WARNING)<<"This mono strategy not implemented yet."<<endl;
                }
            }
            int success_stereo_tracked_count = output_tracked_pts_left.size();
            *success_tracked_stereo_pts_count = success_stereo_tracked_count;//output.
            LOG(INFO)<<"success tracked for frame:"<<pFrame->frame_id<<", cam:"<<cam_index <<",count:"<<*success_tracked_stereo_pts_count<<endl;
            if(success_stereo_tracked_count>15)
            {
                cout<<"stereo track success!"<<endl;
                *p_output_track_success = true;
            }
            visualized_tracked_p2d_and_ordinary_frame_stereo(*pFrame,output_tracked_pts_left,output_tracked_pts_right,cam_index);
        }
        visualized_tracked_p2d_and_ordinary_frame(*pFrame,output_tracked_pts_left,i);
    }








}
#endif
