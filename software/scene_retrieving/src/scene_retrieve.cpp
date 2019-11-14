#include "scene_retrieve.h"


/*SceneRetriever::SceneRetriever(Scene& original_scene_input)
{
    this->original_scene=original_scene;
    this->_init_retriever();
}*/

Scene::Scene()
{
    return;//TODO:fill in init functions.
}


int Scene::getImageCount()
{
    return this->vec_p2d.size();
}


void Scene::saveVoc()
{
    vector<cv::Mat> features = point_desps;
    
    // branching factor and depth levels 
    const int k = 10;
    const int L = 3;
    const WeightingType weight = TF_IDF;
    const ScoringType score = L1_NORM;

    //OrbVocabulary voc(k, L, weight, score);
    Vocabulary voc(k, L, weight, score);
    //BriefVocabulary voc(k,L,weight,score);

    LOG(INFO) << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
    voc.create(features);
    LOG(INFO) << "... done!" << endl;

    LOG(INFO) << "Vocabulary information: " << endl
    << voc << endl << endl;

    // save the vocabulary to disk
    LOG(INFO) << endl << "Saving vocabulary..." << endl;
    voc.save("small_voc.yml.gz");
    LOG(INFO) << "Done" << endl;
}


void Scene::saveFile(const std::string &filename)
{
    std::ofstream ofs(filename);

    {
        boost::archive::text_oarchive oa(ofs);
        oa << *this;
    }
}


void Scene::loadFile(const std::string &filename)
{
    std::ifstream ifs(filename);

    {
        boost::archive::text_iarchive ia(ifs);
        ia >> *this;
    }

    LOG(INFO) << "Deserialization finished" << endl;
    this->test();

}

void Scene::saveDeserializedPoseToCSV()
{
    std::ofstream PoseToFile;
    PoseToFile.open("./loadedpose.csv");
    
    for(auto& m: this->mVecT)
    {
        if (!m.empty())
        {
            LOG(INFO)<<"Saveing translation: "<<m<<endl;
            PoseToFile << to_string(m.at<double>(0)) + "," + to_string(m.at<double>(1)) + "," + to_string(m.at<double>(2)) + "\n";
        }
    }
}

void Scene::test(bool savePosition)
{
    LOG(INFO)<<"---------------Current scene info---------------"<<endl;
    
//    for(auto& m: this->mVecR)
//    {
//        LOG(INFO)<<"VecR is :"<<endl<<m<<endl;
//    }
//
//    for(auto& m: this->mVecT)
//    {
//        LOG(INFO)<<"VecT is :"<<endl<<m<<endl;
//    }
    
    LOG(INFO)<<"mIndex: "<<mIndex<<endl;
    LOG(INFO)<<"hasScale: "<<hasScale<<endl;
    LOG(INFO)<<"vec_p2d size: "<<vec_p2d.size()<<endl;
    LOG(INFO)<<"vec_p3d size: "<<vec_p3d.size()<<endl;
    LOG(INFO)<<"point_desps size: "<<point_desps.size()<<endl;
    LOG(INFO)<<"vec_r size: "<<mVecR.size()<<endl;
    LOG(INFO)<<"vec_t size: "<<mVecT.size()<<endl;
    
    LOG(INFO)<<"---------------Current scene info---------------"<<endl;
    if (savePosition)
    {
        saveDeserializedPoseToCSV();
    }

    ofstream p2d, p3d, vecR, vecT;
    p2d.open("p2d.log");
    p3d.open("p3d.log");
    vecR.open("vecR.log");
    vecT.open("vecT.log");

    for(auto& p2dpts : vec_p2d)
    {
      for(auto& p2dpt : p2dpts)
      {
        p2d << p2dpt.pt.x<<", "<<p2dpt.pt.y <<endl;
      }
    }

    for(auto& p3dpts : vec_p3d)
    {
      for(auto& p3dpt : p3dpts)
      {
        p3d << p3dpt.x<<", "<<p3dpt.y<<", "<<p3dpt.z<<endl;
      }
    }

    p2d.close();
    p3d.close();

}

std::vector <cv::Point3d> Scene::fetchFrameMapPoints(size_t frame_index)
{
    assert(frame_index > 0 && frame_index < this->vec_p3d.size());

    return this->vec_p3d[frame_index];
}

void Scene::removeEmptyElement()
{
    int sum_a=0, sum_b=0, sum_c=0, sum_d=0, sum_e=0;
    for(int i = 0; i<this->vec_p2d.size(); i++)
    {
        if(this->vec_p2d[i].empty())
        {
            this->vec_p2d.erase(this->vec_p2d.begin() + i);
            sum_a ++;
        }
    }

    for(int i = 0; i<this->vec_p3d.size(); i++)
    {
        if(this->vec_p3d[i].empty())
        {
            this->vec_p3d.erase(this->vec_p3d.begin() + i);
            sum_b ++;
        }
    }

    for(int i = 0; i<this->mVecR.size(); i++)
    {
        if(this->mVecR[i].empty())
        {
            this->mVecR.erase(this->mVecR.begin() + i);
            sum_c ++;
        }
    }

    for(int i = 0; i<this->mVecT.size(); i++)
    {
        if(this->mVecT[i].empty())
        {
            this->mVecT.erase(this->mVecT.begin() + i);
            sum_d ++;
        }
    }

    for(int i = 0; i<this->point_desps.size(); i++)
    {
        if(this->point_desps[i].empty())
        {
            this->point_desps.erase(this->point_desps.begin() + i);
            sum_e ++;
        }
    }

    this->mIndex -= sum_a;

    LOG(INFO) <<"Removed Features: "<<sum_a <<endl;
    LOG(INFO) <<"Removed MapPoints: "<<sum_b <<endl;
    LOG(INFO) <<"Removed R: "<<sum_c <<endl;
    LOG(INFO) <<"Removed t: "<<sum_d <<endl;
    LOG(INFO) <<"Removed point_desps: "<<sum_e <<endl;

}



//---------------------------------------class SceneRetriever------------------------------------

SceneRetriever::SceneRetriever(){};
SceneRetriever::SceneRetriever(const string& voc)
{
    this->original_scene = Scene();
    this->ploop_closing_manager_of_scene = shared_ptr<LoopClosingManager>(new LoopClosingManager(voc));
    this->mpCv_helper = shared_ptr<cv_helper>(new cv_helper(376.0, 376.0, 376.0, 240.0, 45.12));
    this->mpCv_helper->setMask("mask.png");

    this->_init_retriever();

}

SceneRetriever::SceneRetriever(const string& voc,const string& scene_file)
{
    this->original_scene.loadFile(scene_file);
    this->ploop_closing_manager_of_scene = shared_ptr<LoopClosingManager>(new LoopClosingManager(voc));

    this->mpCv_helper = shared_ptr<cv_helper>(new cv_helper(376.0, 376.0, 376.0, 240.0, 45.12));
    this->mpCv_helper->setMask("mask.png");


    this->_init_retriever();

    LOG(INFO)<<"Publishing history pose started!"<<endl;
    this->publishPoseHistory();
    LOG(INFO)<<"Publishing history pose finished!"<<endl;

    int image_num = 5000;

    for (int i=0; i<image_num; i++)
    {
        string left_path = "./image/left/" + to_string(i) + ".png";
        mVecLeftImagePath.push_back(left_path);
    }

    mThresFitnessScore = 2.0;

    mMavrosSub = mNH.subscribe("/mavros/vision_pose/pose", 1, &SceneRetriever::MavrosPoseCallback, this);
    //mMavrosSub = mNH.subscribe("/mavros/local_position/pose", 1, &SceneRetriever::MavrosPoseCallback, this);
}

SceneRetriever::SceneRetriever(const string& voc,const string& scene_file, const string config_file)
{
    this->original_scene.loadFile(scene_file);
    this->ploop_closing_manager_of_scene = shared_ptr<LoopClosingManager>(new LoopClosingManager(voc));

    this->mpCv_helper = shared_ptr<cv_helper>(new cv_helper(376.0, 376.0, 376.0, 240.0, 45.12));
    this->mpCv_helper->setMask("mask.png");


    this->_init_retriever();

    LOG(INFO)<<"Publishing history pose started!"<<endl;
    this->publishPoseHistory();
    LOG(INFO)<<"Publishing history pose finished!"<<endl;

    int image_num = 5000;

    for (int i=0; i<image_num; i++)
    {
        string left_path = "./image/left/" + to_string(i) + ".png";
        mVecLeftImagePath.push_back(left_path);
    }

    mMavrosSub = mNH.subscribe("/mavros/vision_pose/pose", 1, &SceneRetriever::MavrosPoseCallback, this);
    //mMavrosSub = mNH.subscribe("/mavros/local_position/pose", 1, &SceneRetriever::MavrosPoseCallback, this);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    mThresFitnessScore = fsSettings["Fitness_Threshold"];
    LOG(INFO)<<"mThresFitnessScore: "<<mThresFitnessScore<<endl;

    LOG(INFO)<<"config_file: "<<config_file<<endl;
}

void SceneRetriever::MavrosPoseCallback(const geometry_msgs::PoseStamped& pose)
{
    mCurMavrosPose = PoseStampedToMat(pose);
}


void SceneRetriever::_init_retriever()
{
    LOG(INFO)<<"Init retriever start!"<<endl;

    auto p2d = this->original_scene.getP2D();
    auto p3d = this->original_scene.getP3D();

    LOG(INFO)<<"SceneRetriever::init retriever start: "<<original_scene.getImageCount()<<endl;

    for(int frame_index = 0; frame_index < original_scene.getImageCount(); frame_index++)
    {

        struct FrameInfo* pfr = new struct FrameInfo;
	    
        pfr->keypoints = p2d[frame_index];
        pfr->descriptors = this->original_scene.getDespByIndex(frame_index);

        LOG(INFO)<<"pfr->keypoints size: "<<pfr->keypoints.size()<<endl;
        LOG(INFO)<<"pfr->descriptors size: "<<pfr->descriptors.size()<<endl;


        ptr_frameinfo frame_info(pfr);
        
        this->ploop_closing_manager_of_scene->addKeyFrame(frame_info);
    }

    LOG(INFO)<<"frameinfo_list.size()"<<this->ploop_closing_manager_of_scene->frameinfo_list.size()<<endl;
    LOG(INFO)<<"loaded database information: "<<this->ploop_closing_manager_of_scene->frame_db<<endl;
    LOG(INFO)<<"Init retriever finished!"<<endl;
}


void SceneRetriever::setImageVecPath(vector<string>& imageVec, int left)
{
    if(left)
        this->mVecLeftImagePath = imageVec;

    if(!left)
        this->mVecRightImagePath = imageVec;
}


cv::Mat SceneRetriever::fetchImage(int index, int left)
{
    LOG(INFO)<<"fetchImage: "<<index<<", "<<left<<endl;
    LOG(INFO)<<"this->mVecLeftImagePath.size(): "<<this->mVecLeftImagePath.size()<<endl;
    LOG(INFO)<<"fetchImage: "<<index<<", "<<left<<endl;
    LOG(INFO)<<"this->mVecLeftImagePath.size(): "<<this->mVecLeftImagePath.size()<<endl;


    if(left)
    {
        if(index >=0 && index < this->mVecLeftImagePath.size())
        {
            LOG(INFO)<<"cv::imread(this->mVecLeftImagePath[index]) size: "<<cv::imread(this->mVecLeftImagePath[index]).size()<<endl;
            return cv::imread(this->mVecLeftImagePath[index]);
        }
    }
    else
    {
        if(index >=0 && index < this->mVecRightImagePath.size())
        {
            return cv::imread(this->mVecRightImagePath[index]);
        }
    }
}


void SceneRetriever::displayFeatureMatches(cv::Mat curImage, vector<cv::KeyPoint> curKps,
                                           cv::Mat oldImage, vector<cv::KeyPoint> oldKps,
                                           std::vector<cv::DMatch> matches, size_t loop_index) {

    cv::Mat output_image;

    if (!curImage.empty() && !oldImage.empty() && !matches.empty() && !curKps.empty() && !oldKps.empty()) {
        cv::drawMatches(curImage, curKps, oldImage, oldKps, matches, output_image);
        cv::putText(output_image, "matched_kps size: " + to_string(matches.size()), cv::Point(20, 60), 2, 2,
                    cv::Scalar(0, 0, 255));
        cv::imwrite("./loopclosure_result/" + std::to_string(this->LoopClosureDebugIndex) + "_" +
                    std::to_string(loop_index) + ".png", output_image);
    }
}

void SceneRetriever::displayFeatureMatches(size_t loop_index, ptr_frameinfo& current_frame_info, std::vector<cv::DMatch> matches)
{

    vector<cv::KeyPoint> cur_keypoints = current_frame_info->keypoints;

    ptr_frameinfo retreived_Frame_info = ploop_closing_manager_of_scene->frameinfo_list[loop_index];

    vector<cv::KeyPoint> old_keypoints = retreived_Frame_info->keypoints;

    cv::Mat output_image;

    cv::drawMatches(this->mCurrentImage, cur_keypoints, fetchImage(loop_index, 1), old_keypoints, matches, output_image);

    if(!output_image.empty())
        cv::imwrite("./loopclosure_result/" + std::to_string(this->LoopClosureDebugIndex) + "_" +std::to_string(loop_index) + ".png", output_image);

}


float SceneRetriever::retrieveSceneFromStereoImage(cv::Mat& image_left_rect, cv::Mat& image_right_rect,
                                                   cv::Mat& mavros_pose, cv::Mat& RT_mat_of_stereo_cam_output,
                                                   bool& match_success, int* pMatchedIndexID_output)
{
    this->LoopClosureDebugIndex ++;

    // mCurMavrosPose could either be from SLAM or GPS
    mavros_pose = mCurMavrosPose;

    if (image_left_rect.empty() || image_right_rect.empty())
    {
        LOG(INFO)<<"Left or Right image are empty, return."<<endl;
        match_success = false;
        return -1;
    }

    this->mCurrentImage = image_left_rect;

    //match left image with scene.
    std::vector<cv::DMatch> good_matches_output;
    ptr_frameinfo frameinfo_left = this->ploop_closing_manager_of_scene->extractFeature(image_left_rect);

    int loop_index= this->ploop_closing_manager_of_scene->detectLoopByKeyFrame(frameinfo_left, good_matches_output, true);

    LOG(INFO)<<"Loop Index: "<<loop_index<<endl;
    if(loop_index<0)
    {
        //frame match failed.
        match_success = false;
        return -1;
    }

    LOG(INFO) << "good_matches_output: " << good_matches_output.size() << endl;

    //step 1, fetch current frame camera points and desps
    vector<cv::KeyPoint> current_kps_left;
    vector<cv::Point3f> current_camera_pts;
    cv::Mat current_frame_desps;
    if(!mpCv_helper->StereoImage2CamPoints(image_left_rect, image_right_rect, current_kps_left, current_camera_pts, current_frame_desps))
    {
        match_success = false;
        return -1;
    }

//    if(!mpCv_helper->StereoImage2CamPointsORB(image_left_rect, image_right_rect, current_kps_left, current_camera_pts, current_frame_desps))
//    {
//        match_success = false;
//        return -1;
//    }

    // NOTE, retrieved elements from boost deserialization
    auto old_kps_left = original_scene.vec_p2d[loop_index];
    auto old_camera_pts_3d = original_scene.vec_p3d[loop_index];
    auto old_camera_pts = pts3dto3f(old_camera_pts_3d);
    auto old_frame_desps = original_scene.point_desps[loop_index];

    //step 3, match current and old features
    vector<cv::DMatch> result_matches;
    mpCv_helper->match2Images(current_kps_left, current_frame_desps,
                              old_kps_left, old_frame_desps,
                              result_matches);


    //step 4, if few matches, return false
    if(result_matches.size() < 20)
    {
        match_success = false;
        return -1;
    }

    //step 5, update matched feature points and camera points
    vector<cv::KeyPoint> matched_current_kps, matched_old_kps;
    vector<cv::Point3f> matched_current_cam_pts, matched_old_cam_pts;

    for(int i=0; i<result_matches.size(); i++)
    {
        matched_current_kps.push_back(current_kps_left[result_matches[i].queryIdx]);
        matched_current_cam_pts.push_back(current_camera_pts[result_matches[i].queryIdx]);

        matched_old_kps.push_back(old_kps_left[result_matches[i].trainIdx]);
        matched_old_cam_pts.emplace_back(float(old_camera_pts[result_matches[i].trainIdx].x),
                                         float(old_camera_pts[result_matches[i].trainIdx].y),
                                         float(old_camera_pts[result_matches[i].trainIdx].z));
    }


    bool debug = false;
    if(debug)
    {
        LOG(INFO)<<"current_kps_left size: "<<current_kps_left.size()<<endl;
        LOG(INFO)<<"old_kps_left size: "<<old_kps_left.size()<<endl;
        LOG(INFO)<<"result_matches size: "<<result_matches.size()<<endl;
        cv::Mat old_image_left = cv::imread(mVecLeftImagePath[loop_index]);
        this->displayFeatureMatches(image_left_rect, current_kps_left,
                                    old_image_left, old_kps_left,
                                    result_matches, loop_index);
    }


    //step 6, now that we have matched camera points we can conduct ICP, we can use either PCL method or opencv method
    // in practical I find GeneralICP is slower than regular ICP
    Eigen::Matrix4f result;
    //float fitnesscore = mpCv_helper->GeneralICP(matched_current_cam_pts, matched_old_cam_pts, result);
    float fitnesscore = mpCv_helper->ICP(matched_current_cam_pts, matched_old_cam_pts, result);

    if(fitnesscore == -1)
    {
      match_success = false;
      return fitnesscore;
    }

    LOG(INFO)<<"get fitness score: "<<fitnesscore<<endl;
    LOG(INFO)<<"icp given old T and relative loop closure T, get new T"<<endl;
    LOG(INFO)<<"Calculated transform matrix is: \n"<<result<<endl;

    //step 7, given old T and relative loop closure T, get new T
    cv::Mat result_relative_T;
    cv::eigen2cv(result, result_relative_T);
    cv::Mat result_R = result_relative_T.colRange(0,3).rowRange(0,3);
    cv::Mat result_t = result_relative_T.rowRange(0,3).col(3);

    //step 8, given old left image and current left image, compute relative R vec from rodrigues by essential mat
    cv::Mat essentialR = mpCv_helper->getRotationfromEssential(matched_current_kps, matched_old_kps);
    cv::Mat essentialRvec, RrelativeVec;
    cv::Rodrigues (essentialR, essentialRvec);
    cv::Rodrigues (result_R, RrelativeVec);

    float distanceR = mpCv_helper->Vec3Distance(essentialRvec, RrelativeVec);

//    if(distanceR>2.0)
//        return -1;

    //step 10, if R vector distance is small enough , consider as inlier and continue to the next step

    //fetch old frame R and t
    cv::Mat R = this->original_scene.getR(loop_index);
    cv::Mat t = this->original_scene.getT(loop_index);

    LOG(INFO)<<"R and t old are: "<<R<<", "<<t<<endl;

    cv::Mat old_T = cv::Mat::zeros(4,4, CV_64FC1);

    old_T.at<double>(0, 0) = R.at<double>(0, 0);
    old_T.at<double>(0, 1) = R.at<double>(0, 1);
    old_T.at<double>(0, 2) = R.at<double>(0, 2);
    old_T.at<double>(1, 0) = R.at<double>(1, 0);
    old_T.at<double>(1, 1) = R.at<double>(1, 1);
    old_T.at<double>(1, 2) = R.at<double>(1, 2);
    old_T.at<double>(2, 0) = R.at<double>(2, 0);
    old_T.at<double>(2, 1) = R.at<double>(2, 1);
    old_T.at<double>(2, 2) = R.at<double>(2, 2);

    old_T.at<double>(0, 3) = t.at<double>(0, 0);
    old_T.at<double>(1, 3) = t.at<double>(1, 0);
    old_T.at<double>(2, 3) = t.at<double>(2, 0);

    old_T.at<double>(3, 3) = 1.0;

    cv::Mat new_T;

    result_relative_T.convertTo(result_relative_T, CV_64F);

    //NOTE previously computed ICP result is relative from current camera pose to fetched camera pose,
    //computed relative ICP is in opencv image frame, x points to right, y points to down and z points up.
    //we need to convert relative ICP results to the old_T frame, which is FLU


    cv::Mat image_to_flu = (cv::Mat_<double>(4,4) << 1, 0, 0, result_relative_T.at<double>(2, 3),
                                                     0, 1, 0, - result_relative_T.at<double>(0, 3),
                                                     0, 0, 1, - result_relative_T.at<double>(1, 3),
                                                     0, 0, 0, 1);

    //NOTE, result relative_T is relative from current to old camera frame, old_T is in global frame.
    //we need to update computed current pose.s
    new_T = old_T * image_to_flu;

    cv::Mat new_R = new_T.colRange(0,3).rowRange(0,3);
    cv::Mat new_t = new_T.rowRange(0,3).col(3);

    LOG(INFO)<<"new_t: "<<new_t<<endl;
    LOG(INFO)<<"fitnesscore: "<<fitnesscore<<endl;
    LOG(INFO)<<"mThresFitnessScore: "<<mThresFitnessScore<<endl;

    if (fitnesscore < mThresFitnessScore)
    {
        this->mpCv_helper->publishPose(new_R, new_t, 0);
        RT_mat_of_stereo_cam_output = new_T;
        match_success = true;

        if(pMatchedIndexID_output != nullptr)
        {
            *pMatchedIndexID_output = loop_index;
        }
    }
    else
    {
        match_success = false;
    }

    return fitnesscore;
}

float SceneRetriever::retrieveSceneFromStereoImage(cv::Mat& image_left_rect, cv::Mat& image_right_rect,
                                                   cv::Mat& mavros_pose, cv::Mat& RT_mat_of_stereo_cam_output, cv::Mat& Q_mat,
                                                   bool& match_success,int* pMatchedIndexID_output)
{
    this->LoopClosureDebugIndex ++;

    // mCurMavrosPose could either be from SLAM or GPS
    mavros_pose = mCurMavrosPose;

    if (image_left_rect.empty() || image_right_rect.empty())
    {
        LOG(INFO)<<"Left or Right image are empty, return."<<endl;
        match_success = false;
        return -1;
    }

    this->mCurrentImage = image_left_rect;

    //match left image with scene.
    std::vector<cv::DMatch> good_matches_output;
    ptr_frameinfo frameinfo_left = this->ploop_closing_manager_of_scene->extractFeature(image_left_rect);

    int loop_index= this->ploop_closing_manager_of_scene->detectLoopByKeyFrame(frameinfo_left, good_matches_output, true);

    LOG(INFO)<<"Loop Index: "<<loop_index<<endl;
    if(loop_index<0)
    {
        //frame match failed.
        match_success = false;
        return -1;
    }

    LOG(INFO) << "good_matches_output: " << good_matches_output.size() << endl;

    //step 1, fetch current frame camera points and desps
    vector<cv::KeyPoint> current_kps_left;
    vector<cv::Point3f> current_camera_pts;
    cv::Mat current_frame_desps;
    if(!mpCv_helper->StereoImage2CamPoints(image_left_rect, image_right_rect, current_kps_left, current_camera_pts, current_frame_desps))
    {
        match_success = false;
        return -1;
    }

    // ------------------------------------------------------------------------------------------------------------------------------------
    // NOTE real time computed elements
//    vector<cv::KeyPoint> old_kps_left;
//    vector<cv::Point3f> old_camera_pts;
//    cv::Mat old_frame_desps;
//    LOG(INFO)<<"old_image_left size: "<<old_image_left.size()<<endl;
//    if(!mpCv_helper->StereoImage2CamPoints(old_image_left, old_image_right, old_kps_left, old_camera_pts, old_frame_desps))
//    {
//        match_success = false;
//        return -1;
//    }

    // NOTE, previously computed elements
    auto old_kps_left = original_scene.vec_p2d[loop_index];
    auto old_camera_pts_3d = original_scene.vec_p3d[loop_index];
    auto old_camera_pts = pts3dto3f(old_camera_pts_3d);
    auto old_frame_desps = original_scene.point_desps[loop_index];
    LOG(INFO)<<"old_kps_left size: "<<old_kps_left.size()<<endl;
    LOG(INFO)<<"old_camera_pts_3d size: "<<old_camera_pts_3d.size()<<endl;
    LOG(INFO)<<"old_frame_desps size: "<<old_frame_desps.size()<<endl;

    //step 3, match current and old features
    vector<cv::DMatch> result_matches;
    mpCv_helper->match2Images(current_kps_left, current_frame_desps,
                              old_kps_left, old_frame_desps,
                              result_matches);


    //step 4, if few matches, return false
    if(result_matches.size() < 20)
    {
        match_success = false;
        return -1;
    }

    //step 5, update matched feature points and camera points
    vector<cv::KeyPoint> matched_current_kps, matched_old_kps;
    vector<cv::Point3f> matched_current_cam_pts, matched_old_cam_pts;

    for(int i=0; i<result_matches.size(); i++)
    {
        matched_current_kps.push_back(current_kps_left[result_matches[i].queryIdx]);
        matched_current_cam_pts.push_back(current_camera_pts[result_matches[i].queryIdx]);

        matched_old_kps.push_back(old_kps_left[result_matches[i].trainIdx]);
        matched_old_cam_pts.emplace_back(float(old_camera_pts[result_matches[i].trainIdx].x),
                                         float(old_camera_pts[result_matches[i].trainIdx].y),
                                         float(old_camera_pts[result_matches[i].trainIdx].z));
    }


    //step 6, now that we have matched camera points we can conduct ICP, we can use either PCL method or opencv method
    Eigen::Matrix4f result;
    float fitnesscore = mpCv_helper->GeneralICP(matched_current_cam_pts, matched_old_cam_pts, result);
    //float fitnesscore = mpCv_helper->ICP(matched_current_cam_pts, matched_old_cam_pts, result);

    if(fitnesscore == -1)
    {
        match_success = false;
        return fitnesscore;
    }

    LOG(INFO)<<"get fitness score: "<<fitnesscore<<endl;
    LOG(INFO)<<"icp given old T and relative loop closure T, get new T"<<endl;
    LOG(INFO)<<"Calculated transform matrix is: \n"<<result<<endl;

    //step 7, given old T and relative loop closure T, get new T
    cv::Mat result_relative_T;
    cv::eigen2cv(result, result_relative_T);
    cv::Mat result_R = result_relative_T.colRange(0,3).rowRange(0,3);
    cv::Mat result_t = result_relative_T.rowRange(0,3).col(3);

    //step 10, if R vector distance is small enough , consider as inlier and continue to the next step

    //fetch old frame R and t
    cv::Mat R = this->original_scene.getR(loop_index);
    cv::Mat t = this->original_scene.getT(loop_index);

    LOG(INFO)<<"R and t old are: "<<R<<", "<<t<<endl;

    cv::Mat old_T = cv::Mat::zeros(4,4, CV_64FC1);

    old_T.at<double>(0, 0) = R.at<double>(0, 0);
    old_T.at<double>(0, 1) = R.at<double>(0, 1);
    old_T.at<double>(0, 2) = R.at<double>(0, 2);
    old_T.at<double>(1, 0) = R.at<double>(1, 0);
    old_T.at<double>(1, 1) = R.at<double>(1, 1);
    old_T.at<double>(1, 2) = R.at<double>(1, 2);
    old_T.at<double>(2, 0) = R.at<double>(2, 0);
    old_T.at<double>(2, 1) = R.at<double>(2, 1);
    old_T.at<double>(2, 2) = R.at<double>(2, 2);

    old_T.at<double>(0, 3) = t.at<double>(0, 0);
    old_T.at<double>(1, 3) = t.at<double>(1, 0);
    old_T.at<double>(2, 3) = t.at<double>(2, 0);

    old_T.at<double>(3, 3) = 1.0;

    cv::Mat new_T;

    result_relative_T.convertTo(result_relative_T, CV_64F);

    //NOTE previously computed ICP result is relative from current camera pose to fetched camera pose,
    //computed relative ICP is in opencv image frame, x points to right, y points to down and z points up.
    //we need to convert relative ICP results to the old_T frame, which is FLU
    cv::Mat image_to_flu = (cv::Mat_<double>(4,4) << 0, 0, 1, 0,
                                                    -1, 0, 0, 0,
                                                    0, -1, 0, 0,
                                                    0, 0, 0, 1);

    //NOTE, result relative_T is relative from current to old camera frame, old_T is in global frame.
    //we need to update computed current pose.

    new_T = old_T * (image_to_flu * result_relative_T);
    //new_T = old_T * result_relative_T;

    cv::Mat new_R = new_T.colRange(0,3).rowRange(0,3);
    cv::Mat new_t = new_T.rowRange(0,3).col(3);

    LOG(INFO)<<"old t: "<<t<<endl;
    LOG(INFO)<<"result_t: "<<result_t<<endl;
    LOG(INFO)<<"new_t: "<<new_t<<endl;

    if (fitnesscore < mThresFitnessScore)
    {
        this->mpCv_helper->publishPose(new_R, new_t, 0);
        RT_mat_of_stereo_cam_output = new_T;
        match_success = true;

        if(pMatchedIndexID_output != nullptr)
        {
            *pMatchedIndexID_output = loop_index;
        }
    }
    else
    {
        match_success = false;
    }

    return fitnesscore;
}



float SceneRetriever::retrieveSceneWithScaleFromMonoImage(cv::Mat image_left_rect, cv::Mat& cameraMatrix, cv::Mat& RT_mat_of_mono_cam_output, bool& match_success,int* pMatchedIndexID_output)
{
  //    LOG(INFO)<<"in retrieveSceneWithScaleFromMonoImage()."<<endl;
  //    if(this->original_scene.hasScale == false)
  //    {
  //        LOG(ERROR)<<"ERROR in retrieveSceneWithScaleFromMonoImage():scene is set with no scale.check your config!!!!"<<endl;
  //        match_success = false;
  //        return -1;
  //    }
  //
  //    // NOTE this is the working version of SHR
  //    this->LoopClosureDebugIndex ++;
  //
  //    //step<1> generate sparse pointcloud of image pair input and scene.
  //    if (image_left_rect.empty())
  //    {
  //        LOG(WARNING)<<"Left or Right image is empty, return."<<endl;
  //        match_success = false;
  //        return -1;
  //    }
  //    LOG(INFO)<<"in retrieveSceneWithScaleFromMonoImage(): will applyMask()."<<endl;
  //    // apply mask to input image
  //    //mpCv_helper->applyMask(image_left_rect);
  //
  ////    cv::imshow("left image", image_left_rect);
  ////    cv::waitKey(5);
  //
  //    this->mCurrentImage = image_left_rect;
  //
  //    //<1>-(1) match left image with scene.
  //    std::vector<cv::DMatch> good_matches_output;
  //    LOG(INFO)<<"in retrieveSceneWithScaleFromMonoImage:Extracting feature."<<endl;
  //    ptr_frameinfo frameinfo_left = this->ploop_closing_manager_of_scene->extractFeature(image_left_rect);
  //
  //    LOG(INFO)<<"in retrieveSceneWithScaleFromMonoImage:detecting loop."<<endl;
  //    int loop_index= this->ploop_closing_manager_of_scene->detectLoopByKeyFrame(frameinfo_left, good_matches_output, true);
  //
  //    if(loop_index<0)
  //    {
  //        match_success = false;
  //        return -1;
  //    }
  //
  //    LOG(INFO)<<"find loop.loop Index: "<<loop_index<<endl;
  //    //NOTE display feature matches between current frame and detected old frame
  //    LOG(INFO) << "good_matches_output: " << good_matches_output.size() << endl;
  //    LOG(INFO) << "good_matches_output: " << good_matches_output.size() << endl;
  //
  //    if (good_matches_output.size() > 10) {
  //        ;//this->displayFeatureMatches(loop_index, frameinfo_left, good_matches_output); //TODO: ?????
  //    }
  //
  //    //fetch left and right image
  //    cv::Mat old_image_left = this->fetchImage(loop_index, 1);
  //    cv::Mat old_image_right = this->fetchImage(loop_index, 0);
  //
  //    //fetch old frame R and t
  //    cv::Mat R = this->original_scene.getR(loop_index);
  //    cv::Mat t = this->original_scene.getT(loop_index);
  //
  //    //initialize PnP result
  //    cv::Mat result_R, result_t;
  //
  //    //conduct pnp
  //    if (old_image_right.empty() || old_image_left.empty() || R.empty() || t.empty())
  //    {
  //        LOG(INFO)<<"Image empty or Rt empty before solvePnP!"<<endl;
  //        LOG(INFO)<<"l,r,R,t:"<<old_image_left<<";"<<old_image_right<<";"<<R<<";"<<t<<endl;
  //        LOG(INFO)<<"Image empty or Rt empty before solvePnP"<<endl;
  //        match_success = false;
  //        return -1;
  //    }
  //    else
  //    {
  //        int pnpResult = this->mpCv_helper->solvePnP(old_image_left,
  //                                                     old_image_right,
  //                                                     image_left_rect,
  //                                                     R, t,
  //                                                     result_R, result_t);
  //
  //        LOG(INFO)<<"pnpresult: "<<pnpResult<<endl;
  //        LOG(INFO)<<"pnpresult: "<<bool(pnpResult)<<endl;
  //        if(pnpResult>0 && !result_R.empty() && !result_t.empty())
  //        {
  //            LOG(INFO)<<"pnp result are: \n: "<<result_R<<endl<<result_t<<endl;
  //            LOG(INFO)<<"solve pnp finished, publishing the result."<<endl;
  //
  //            cv::Mat temp_t =  -result_R.t()*result_t;
  //
  //            //this->mpCv_helper->publishPose(result_R, result_t, 0);
  //            this->mpCv_helper->publishPose(-result_R.t(), temp_t, 0);
  //
  //            LOG(INFO)<<"solve pnp finished, publishing the result finished."<<endl;
  //            LOG(INFO)<<"Match success in retrieveSceneWithScaleFromMonoImage()!"<<endl;
  //            match_success = true;
  //            if(pMatchedIndexID_output!=nullptr)
  //            {
  //                *pMatchedIndexID_output = loop_index;
  //            }
  //            return pnpResult;
  //        }
  //        else
  //        {
  //            match_success = false;
  //            return -1;
  //        }
  //
  //    }
  //


    // NOTE this is the original version of WHY
    match_success = false;
    cv::Mat& image_in_rect = image_left_rect;
    ptr_frameinfo mono_image_info = this->ploop_closing_manager_of_scene->extractFeature(image_in_rect);
    std::vector <cv::DMatch> good_matches;

    int loop_index = this->ploop_closing_manager_of_scene->detectLoopByKeyFrame(mono_image_info,good_matches,true);//false); // the last parameter decrepted.WTF is that???

    if (loop_index<0)
    {
        LOG(INFO)<<"    in retrieveSceneWithScaleFromMonoImage():loop_index <0;detect loop failed!"<<endl;
        return -1;//Loop not found!
    }
    LOG(INFO)<<"    in retrieveSceneWithScaleFromMonoImage():loop_index:"<<loop_index<<";detect loop success!"<<endl;

    //query 3d points and do 2d-3d matching by PnP ransac.
    std::vector<cv::Point3f> points3d;
    std::vector<cv::Point2f> projectedPoints2d;
    LOG(INFO)<<"    generating good_match p2d ,p3d..."<<endl;
    LOG(INFO)<<"    mono_image_info->keypoints.size():"<<mono_image_info->keypoints.size()<<endl;
    for(auto &match:good_matches)
    {
        LOG(INFO)<<"    deref mono_image_info->keypoints at index:"<<match.queryIdx<<endl;
        cv::Point2f pt1 = mono_image_info->keypoints[match.queryIdx].pt;//image point 2d.
        projectedPoints2d.push_back(pt1);
        LOG(INFO)<<"    p3d at loop index:"<<loop_index<<endl;
        LOG(INFO)<<"    p3d size:"<<this->original_scene.getP3D().at(loop_index).size()<<";deref p3d index:"<<match.trainIdx<<endl;
        cv::Point3f pt2 = this->original_scene.getP3D().at(loop_index)[match.trainIdx];//scene point 3d.
		    //image point 2d of scene: this->loop_closing_manager_of_scene.getFrameInfoById(loop_index)->keypoints[match.trainIdx].pt;
        points3d.push_back(pt2);
    }

    //cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_32FC1);
    cv::Mat rvec,tvec,inliers;
    LOG(INFO)<<"    preparing for solvePnPRansac()..."<<endl;
    LOG(INFO)<<"    points3d,projectedPoints2d size:"<<points3d.size()<<","<<projectedPoints2d.size()<<endl;
    if(points3d.size() == 0 || projectedPoints2d.size() == 0)
    {
        LOG(INFO)<<"    before cv::solvePnPRansac(): 3d or 2d points vec size ==0.Failed.Abort."<<endl;
        return -1;
    }
    bool cv_solvepnpransac_result = cv::solvePnPRansac (points3d,//3d
        projectedPoints2d,//2d
        cameraMatrix,
        distCoeffs,
        rvec,
        tvec,
        //bool useExtrinsicGuess =
        false,
        //int iterationsCount =
	100,
        //float reprojectionError =
	8.0,
        //double  confidence =
	0.99,
        inliers,
        //int flags =
	cv::SOLVEPNP_ITERATIVE
        );
    //check inliers.
    cv::Mat R;
    if(cv_solvepnpransac_result )//&& inliers.size()>8)
    {
      LOG(INFO)<<"    in retrieveSceneWithScaleFromMonoImage():Ransac success!!rvec,tvec"<<rvec<<";"<<tvec<<endl;
      cv::Rodrigues(rvec,R);//match success
      LOG(INFO)<<"    in retrieveSceneWithScaleFromMonoImage():after rodrigues R:"<<R<<";"<<tvec<<endl;
      //cv::Mat retMat = cv::Mat::eye(4,4,CV_32F);
      //retMat.convertTo(retMat, CV_64F);
      cv::Mat retMat = cv::Mat(4, 4, R.type(), cv::Scalar(0)); 
      R.copyTo(retMat.rowRange(0,3).colRange(0,3)); 
      //retMat.rowRange(0,3).colRange(0,3) = R;
      //retMat.colRange(3,1).rowRange(0,3) = tvec;
      LOG(INFO)<<"    in retrieveSceneWithScaleFromMonoImage():after replacing retMat,will write retMat into RT_mat_of_mono_cam_output."<<endl;
      RT_mat_of_mono_cam_output = retMat;
      match_success = true;
      if(pMatchedIndexID_output!=nullptr)
      {
          *pMatchedIndexID_output = loop_index;
      }
      LOG(INFO)<<"    in retrieveSceneWithScaleFromMonoImage():return true."<<endl;
      return loop_index;
    }
    else
    {
        LOG(INFO)<<"    in retrieveSceneWithScaleFromMonoImage():Ransac failed!!"<<endl;
        return -1;
    }
    //return.
    
}


void SceneRetriever::publishPoseHistory()
{
    assert(this->mpCv_helper != nullptr);

    vector<cv::Mat> vecT = this->original_scene.mVecT;

    int posenum = this->original_scene.mVecT.size();
    for(int i=0; i<posenum; i++)
    {

        auto t = this->original_scene.mVecT[i];

        if(t.empty())
            continue;

        LOG(INFO)<<"publishing pose: "<<t<<endl;

        cv::Mat R = cv::Mat::ones(3, 3, CV_32F);
        this->mpCv_helper->publishPose(R, t);
    }

//    for(auto& t: this->original_scene.mVecT)
//    {
//        if(t.empty())
//            continue;
//
//        LOG(INFO)<<"publishing pose: "<<t<<endl;
//
//        cv::Mat R = cv::Mat::ones(3, 3, CV_32F);
//        this->mpCv_helper->publishPose(R, t);
//    }
}
