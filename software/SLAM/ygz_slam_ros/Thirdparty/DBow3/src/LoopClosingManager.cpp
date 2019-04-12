#include "../src/LoopClosingManager.h"

bool match_2_images_flann(ptr_frameinfo current_frame,int index2,int &save_index,double score,const std::vector<ptr_frameinfo>& frameinfo_list,std::vector<DMatch>& good_matches_output);
void saveImagePair(int id1,int id2,int &save_index,std::vector<DMatch>& good_matches,double score, const std::vector<ptr_frameinfo> frameinfo_list);


LoopClosingManager::LoopClosingManager(const std::string &voc_path)
{
    this->frame_index = 0;
    this->loop_id = 0;
    //this->orb = cv::ORB::create();
    this->loadVoc(voc_path);
    this->frame_db = Database(this->voc,false,0);
}
LoopClosingManager::LoopClosingManager(const std::string &voc_path,const std::string &frame_db_path)
{
    this->frame_index = 0;
    this->loop_id = 0;
    //this->orb = cv::ORB::create();
    this->loadVoc(voc_path);
    this->frame_db = Database(frame_db_path.c_str());
}

void LoopClosingManager::addKeyFrame(ptr_frameinfo info)
{
    frame_db.add(info->descriptors);
    frameinfo_list.push_back(info);
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
    this->frame_db.query(info->descriptors,results,RET_QUERY_LEN,this->frame_index - TOO_CLOSE_THRES);
    return results;
}
int LoopClosingManager::detectLoopByKeyFrame(ptr_frameinfo info,std::vector<DMatch>& good_matches_output,bool current_frame_has_index = true)
{
    int ret_index = -1;// Not found!
    QueryResults results= this->queryKeyFrames(info);
    for(int wind_index =0;wind_index<results.size();wind_index++)
    {
        if (results[wind_index].Score>DB_QUERY_SCORE_THRES)
        {
	  if (current_frame_has_index)
	  
          if ((current_frame_has_index && results[wind_index].Id<this->frame_index-TOO_CLOSE_THRES) || (!current_frame_has_index))
          {
            // check if this loop candidate satisfies Epipolar Geometry constrain.
	    std::vector<DMatch> matches_out;
            if (match_2_images_flann(info,results[wind_index].Id,this->loop_id,results[wind_index].Score,this->frameinfo_list,matches_out))
            {
	      if (current_frame_has_index)
	      {
                  cout<<"Loop between ["<<this->frame_index<<"\t"<<results[wind_index].Id<<"]"<<endl;
	      }
	      ret_index = results[wind_index].Id;
              break; // match one frame only once.
            }
          }
        }
        else
        {
          break;
        }
    }
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

}

ptr_frameinfo LoopClosingManager::extractFeature(const cv::Mat& image)
{
    cv::Mat mask;
    auto pframeinfo = shared_ptr<FrameInfo>(new FrameInfo);
    //ptr_frameinfo->keypoints
    //vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::ORB> orb;
    orb = cv::ORB::create();
    orb->detectAndCompute(image, mask, pframeinfo->keypoints,pframeinfo->descriptors);
    //orb->detect(image,keypoints);
    //brief->compute(image,keypoints,descriptors);
    //surf->detectAndCompute(image,mask,keypoints,descriptors);
    //surf->compute(image,keypoints,descriptors);
    //kaze->detectAndCompute(image,mask,keypoints,descriptors);

    //features.push_back(vector<cv::Mat >());
    return pframeinfo;
}

void saveImagePair(int id1,int id2,int &save_index,std::vector<DMatch>& good_matches,double score, const std::vector<ptr_frameinfo> frameinfo_list)
{
    stringstream ss;
    ss << "images/image" << id1 << ".png"; 
    cv::Mat image = cv::imread(ss.str(), 0);
    
    stringstream ss2;
    ss2<< "images/image"<<id2<<".png";
    cv::Mat image2 = cv::imread(ss2.str(),0);
    cv::Mat merged_img;
//    cv::hconcat(image,image2,merged_img);
//use cv::drawMatches replace hconcat.
    cv::drawMatches(image,frameinfo_list[id1]->keypoints,image2,frameinfo_list[id2]->keypoints,good_matches,merged_img);
    stringstream output_ss;
    output_ss<<"loops/image"<<save_index<<"__"<<id1<<"_"<<id2<<"___"<<score<<".png";
    cv::imwrite(output_ss.str(),merged_img);
    save_index++;
}

bool match_2_images_flann(ptr_frameinfo current_frame,int index2,int &save_index,double score,const std::vector<ptr_frameinfo>& frameinfo_list,std::vector<DMatch>& good_matches_output)
{
    //TODO:refer VINS KeyFrame::findConnection().

    FlannBasedMatcher matcher = FlannBasedMatcher(makePtr<flann::LshIndexParams>(12,20,2));
    //FlannBasedMatcher matcher = FlannBasedMatcher();
    std::vector< DMatch > matches;
    //matcher.match( kdesp_list[index1], kdesp_list[index2], matches );
    matcher.match(current_frame->descriptors, frameinfo_list[index2]->descriptors, matches);

    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < matches.size(); i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    cout<<"min_dist:"<<min_dist<<endl;
    std::vector< DMatch > good_matches;

    for( int i = 0; i < matches.size(); i++ )
    {
        if( matches[i].distance <= 2*min_dist && matches[i].distance< ORB_TH_HIGH) // 3.0 too large;2.0 too large.
        {
        good_matches.push_back( matches[i]); 
        }
    }
    std::vector<cv::Point2f> match_points1;
    std::vector<cv::Point2f> match_points2;


    cout<<"Good matches count:"<<good_matches.size()<<"."<<endl;
    if(good_matches.size()<STEP1_KP_NUM) // 8 -> 12
    {
        cout<<"Good matches count:"<<good_matches.size()<<"< 8,MATCH FAILED."<<endl;
        return false;
    }
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //kp_list[index1][ good_matches[i].queryIdx ].pt );
        match_points1.push_back( current_frame->keypoints[good_matches[i].queryIdx].pt);
        //kp_list[index2][ good_matches[i].trainIdx ].pt );
        match_points2.push_back( frameinfo_list[index2]->keypoints[good_matches[i].trainIdx].pt);
    }

    Mat isOutlierMask;
    Mat fundamental_matrix = findFundamentalMat(match_points1, match_points2, FM_RANSAC, 3, 0.99,isOutlierMask);
    int final_good_matches_count = 0;
    std::vector<DMatch> final_good_matches;
    for(int i = 0;i<good_matches.size();i++)
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
        cout<<"MATCH SUCCESS."<<endl;
	good_matches_output = final_good_matches;
        return true;
    }
    final_good_matches.clear();
    good_matches_output = final_good_matches;
    return false;
}


