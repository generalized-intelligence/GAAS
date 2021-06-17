#include "../src/LoopClosingManager.h"

bool match_2_images_flann(ptr_frameinfo current_frame,int index2,int &save_index,double score,const std::vector<ptr_frameinfo>& frameinfo_list,std::vector<cv::DMatch>& good_matches_output);
void saveImagePair(int id1,int id2,int &save_index,std::vector<cv::DMatch>& good_matches,double score, const std::vector<ptr_frameinfo> frameinfo_list);


LoopClosingManager::LoopClosingManager(const std::string &voc_path)
{
    this->frame_index = 0;
    this->loop_id = 0;
    //this->orb = cv::ORB::create();
    this->loadVoc(voc_path);
    this->frame_db = DBoW3::Database(this->voc, false, 0);
}

LoopClosingManager::LoopClosingManager(const std::string &voc_path,const std::string &frame_db_path)
{
    this->frame_index = 0;
    this->loop_id = 0;
    //this->orb = cv::ORB::create();
    this->loadVoc(voc_path);
    this->frame_db = DBoW3::Database(frame_db_path.c_str());
}



void LoopClosingManager::addKeyFrame(const ptr_frameinfo& info)
{
//    if (!info->descriptors.empty())
//    {
//        this->frame_db.add(info->descriptors);
//        frameinfo_list.push_back(info);
//        this->frame_index++;
//    }

    this->frame_db.add(info->descriptors);
    this->frameinfo_list.push_back(info);
    this->frame_index++;
}


void LoopClosingManager::addKeyFrame(const cv::Mat& image)
{
    ptr_frameinfo info = extractFeature(image);
    this->addKeyFrame(info);
}


QueryResults LoopClosingManager::queryKeyFrames(ptr_frameinfo info)
{
    QueryResults results;
    //this->frame_db.query(info->descriptors, results, RET_QUERY_LEN, this->frame_index - TOO_CLOSE_THRES);

    this->frame_db.query(info->descriptors, results, 4, this->curFrameIndex - TOO_CLOSE_THRES);
    //this->frame_db.query(info->descriptors, results, 4, -1);

    return results;
}


int LoopClosingManager::detectLoopByKeyFrame(ptr_frameinfo info, std::vector<cv::DMatch>& good_matches_output, bool current_frame_has_index = true)
{
    int ret_index = -1; // Not found!

    QueryResults results= this->queryKeyFrames(info);

    std::vector<cv::DMatch> matches_out;

    //NOTE method 1 will introduce too many outliers

    //iterate four matches
    //-------------------------------------------------------NOTE method 1-------------------------------------------------------
    vector<int> VecMatchSizes;
    vector<int> VecIDs;
    vector<vector<cv::DMatch> > VecMatches;

    for(int wind_index =0; wind_index<results.size(); wind_index++)
    {
        //results[wind_index].Score>DB_QUERY_SCORE_THRES
        if (results[wind_index].Score>0.05)
        {
            if (current_frame_has_index)
            //if (current_frame_has_index && ( std::abs(results[wind_index].Id - this->curFrameIndex) > TOO_CLOSE_THRES) )
            {
                // check if this loop candidate satisfies Epipolar Geometry constrain.
                if (match_2_images_flann(info, results[wind_index].Id, this->loop_id, results[wind_index].Score, this->frameinfo_list, matches_out))
                {
                    cout<<"Loop between [ currenf Frame: "<<this->curFrameIndex<<"\t old_frame: "<<results[wind_index].Id<<"]"<<endl;


                    int current_match_size = matches_out.size();
                    ret_index = results[wind_index].Id;

//                    VecMatchSizes.push_back(current_match_size);
//                    VecIDs.push_back(ret_index);
//                    VecMatches.push_back(matches_out);
                }
            }
        }
        else
        {
            continue;
        }
    }


    //-------------------------------------------------------NOTE method 1-------------------------------------------------------


    //NOTE method 2
//    if(results.size()>0)
//    {
//        //results[wind_index].Score>DB_QUERY_SCORE_THRES
//        //if (results[0].Score>0.08) // works OK
//        //if (results[0].Score>0.1) // a bit too strict
//        //if (results[0].Score>0.05) // too much outliers
//        //if (results[0].Score>0.07) //works OK
//        //if (results[0].Score>0.005) //with EssentialMat outlier detection, works ok with Rotation distance less than 1.5
//        //if (results[0].Score>0.001) //with EssentialMat outlier detection, works ok with Rotation distance less than 2.5
//        if (results[0].Score>0.05)
//        {
//            if (current_frame_has_index)
//                //if (current_frame_has_index && ( std::abs(results[wind_index].Id - this->curFrameIndex) > TOO_CLOSE_THRES) )
//            {
//                // check if this loop candidate satisfies Epipolar Geometry constrain.
//                if (match_2_images_flann(info, results[0].Id, this->loop_id, results[0].Score, this->frameinfo_list, matches_out))
//                {
//                    //cout<<"Loop between ["<<this->frame_index<<"\t"<<results[wind_index].Id<<"]"<<endl;
//                    cout<<"Loop between [ currenf Frame: "<<this->curFrameIndex<<"\t old_frame: "<<results[0].Id<<"]"<<endl;
//
//                    ret_index = results[0].Id;
//                }
//            }
//        }
//    }




    this->curFrameIndex ++;
    good_matches_output = matches_out;
    
    return ret_index;
}


int LoopClosingManager::saveDB()
{
    // Reserve:
    
    // Database frame_db;
    // int frame_index;
    // std::vector<ptr_frameinfo> frameinfo_list;
    
    
}

void LoopClosingManager::loadFromDB()
{
    
}

int LoopClosingManager::loadVoc(const std::string &voc_path)
{
    Vocabulary loaded_voc(voc_path);
    this->voc = loaded_voc;
}


ptr_frameinfo LoopClosingManager::extractFeature(const cv::Mat& image)
{
    cv::Mat mask;
    auto pframeinfo = shared_ptr<FrameInfo>(new FrameInfo);

    cv::Ptr<cv::ORB> orb;
    orb = cv::ORB::create();
    orb->detectAndCompute(image, mask, pframeinfo->keypoints, pframeinfo->descriptors);

    //orb->detect(image,keypoints);
    //brief->compute(image,keypoints,descriptors);
    //surf->detectAndCompute(image,mask,keypoints,descriptors);
    //surf->compute(image,keypoints,descriptors);
    //kaze->detectAndCompute(image,mask,keypoints,descriptors);

    //features.push_back(vector<cv::Mat >());
    return pframeinfo;
}

void saveImagePair(int id1,int id2,int &save_index,std::vector<cv::DMatch>& good_matches,double score, const std::vector<ptr_frameinfo> frameinfo_list)
{
    stringstream ss;
    ss << "images/image" << id1 << ".png"; 
    cv::Mat image = cv::imread(ss.str(), 0);
    
    stringstream ss2;
    ss2<< "images/image"<<id2<<".png";
    cv::Mat image2 = cv::imread(ss2.str(),0);
    cv::Mat merged_img;

    cv::drawMatches(image,frameinfo_list[id1]->keypoints,image2,frameinfo_list[id2]->keypoints,good_matches,merged_img);
    stringstream output_ss;
    output_ss<<"loops/image"<<save_index<<"__"<<id1<<"_"<<id2<<"___"<<score<<".png";
    cv::imwrite(output_ss.str(),merged_img);
    save_index++;
}

bool match_2_images_flann(ptr_frameinfo current_frame, int index2, int &save_index, double score, const std::vector<ptr_frameinfo>& frameinfo_list, std::vector<cv::DMatch>& good_matches_output)
{
    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12,20,2));
    //cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::IndexParams>(12,20,2));
    //cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher();

    std::vector< cv::DMatch > matches;
    //matcher.match( kdesp_list[index1], kdesp_list[index2], matches );


    cout<<"frameinfo_list[index2]->descriptors size: "<<frameinfo_list[index2]->descriptors.size()<<endl;
    cout<<"frameinfo_list[index2]->keypoints size: "<<frameinfo_list[index2]->keypoints.size()<<endl;

    cout<<"current_frame->descriptors size: "<<current_frame->descriptors.size()<<endl;
    cout<<"current_frame->keypoints size: "<<current_frame->keypoints.size()<<endl;

    if(index2<frameinfo_list.size() && !current_frame->descriptors.empty()) {
        matcher.match(current_frame->descriptors, frameinfo_list[index2]->descriptors, matches);
    }

    //GMS
//    vector<cv::DMatch> matches_gms;
//    int GMS_Feature_Matches = 0;
//
//    std::vector<bool> vbInliers;
//    gms_matcher gms(current_frame->keypoints, cv::Size(200, 200), frameinfo_list[index2]->keypoints, cv::Size(200, 200), matches);
//
//    int num_inliers = gms.GetInlierMask(vbInliers, false, false);
//    cout << "GMS Get total " << num_inliers << " matches." << endl;
//
//    // collect matches
//    for (size_t i = 0; i < vbInliers.size(); ++i)
//    {
//        if (vbInliers[i] == true)
//        {
//            cout<<"??? 1"<<endl;
//            matches_gms.push_back(matches[i]);
//            GMS_Feature_Matches ++;
//            cout<<"??? 2"<<endl;
//        }
//    }
//
//
//    matches = matches_gms;

//    if(matches.size()<30)
//        return false;

    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < matches.size(); i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    
    cout<<"min_dist:"<<min_dist<<endl;
    std::vector< cv::DMatch > good_matches;

    cout<<"raw matches size: "<<matches.size()<<endl;


    for( int i = 0; i < matches.size(); i++ )
    {
        if( matches[i].distance <= 2*min_dist && matches[i].distance< 15) // 3.0 too large;2.0 too large.
        {
            good_matches.push_back( matches[i]);
        }
    }

    std::vector<cv::Point2f> match_points1;
    std::vector<cv::Point2f> match_points2;

    cout<<"Good matches size:"<<good_matches.size()<<"."<<endl;

    if(good_matches.size()<STEP1_KP_NUM) // 8 -> 12
    {
        cout<<"Good matches count:"<<good_matches.size()<<"< 8,MATCH FAILED."<<endl;
        return false;
    }
    
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        match_points1.push_back( current_frame->keypoints[good_matches[i].queryIdx].pt );
        match_points2.push_back( frameinfo_list[index2]->keypoints[good_matches[i].trainIdx].pt );
    }


//    good_matches_output = good_matches;
//    return true;


    cout<<"match_2_images_flann, step 2 sizes are: "<<match_points1.size()<<", "<<match_points2.size()<<endl;

    cv::Mat isOutlierMask;
    cv::Mat fundamental_matrix = findFundamentalMat(match_points1, match_points2, cv::FM_RANSAC, 3, 0.99, isOutlierMask);

    int final_good_matches_count = 0;
    std::vector<cv::DMatch> final_good_matches;
    for(int i = 0; i<good_matches.size(); i++)
    {
        if (isOutlierMask.at<int>(i)!=0)
        {
            final_good_matches.push_back(good_matches[i]);
            final_good_matches_count++;
        }
    }



    if(final_good_matches_count>STEP2_KP_NUM)
    {
        //saveImagePair(index1,index2,save_index,final_good_matches,score, frameinfo_list);  //for debug only.
        cout<<"MATCH SUCCEED!"<<endl;
        good_matches_output = final_good_matches;
        return true;
    }

    final_good_matches.clear();
    good_matches_output = final_good_matches;
    return false;

}


