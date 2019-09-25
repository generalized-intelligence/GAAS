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

    cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
    voc.create(features);
    cout << "... done!" << endl;

    cout << "Vocabulary information: " << endl
    << voc << endl << endl;

    // save the vocabulary to disk
    cout << endl << "Saving vocabulary..." << endl;
    voc.save("small_voc.yml.gz");
    cout << "Done" << endl;
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

    cout << "Deserialization finished" << endl;
    this->test();

//    this->removeEmptyElement();
}

void Scene::saveDeserializedPoseToCSV()
{
    std::ofstream PoseToFile;
    PoseToFile.open("./loadedpose.csv");
    
    for(auto& m: this->mVecT)
    {
        if (!m.empty())
        {
            cout<<"Saveing translation: "<<m<<endl;
            PoseToFile << to_string(m.at<double>(0)) + "," + to_string(m.at<double>(1)) + "," + to_string(m.at<double>(2)) + "\n";
        }
    }
}

void Scene::test(bool savePosition)
{
    cout<<"---------------Current scene info---------------"<<endl;
    
//    for(auto& m: this->mVecR)
//    {
//        cout<<"VecR is :"<<endl<<m<<endl;
//    }
//
//    for(auto& m: this->mVecT)
//    {
//        cout<<"VecT is :"<<endl<<m<<endl;
//    }
    
    cout<<"mIndex: "<<mIndex<<endl;
    cout<<"hasScale: "<<hasScale<<endl;
    cout<<"vec_p2d size: "<<vec_p2d.size()<<endl;
    cout<<"vec_p3d size: "<<vec_p3d.size()<<endl;
    cout<<"point_desps size: "<<point_desps.size()<<endl;
    cout<<"vec_r size: "<<mVecR.size()<<endl;
    cout<<"vec_t size: "<<mVecT.size()<<endl;
    
    cout<<"---------------Current scene info---------------"<<endl;
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

    cout <<"Removed Features: "<<sum_a <<endl;
    cout <<"Removed MapPoints: "<<sum_b <<endl;
    cout <<"Removed R: "<<sum_c <<endl;
    cout <<"Removed t: "<<sum_d <<endl;
    cout <<"Removed point_desps: "<<sum_e <<endl;

}



//---------------------------------------class SceneRetriever------------------------------------

SceneRetriever::SceneRetriever(){};
SceneRetriever::SceneRetriever(const string& voc)
{
    this->original_scene = Scene();
    this->ploop_closing_manager_of_scene = shared_ptr<LoopClosingManager>(new LoopClosingManager(voc));
    this->mpCv_helper = shared_ptr<cv_helper>(new cv_helper(360.0652, 363.2195, 406.6650, 256.2053, 39.9554));
    this->mpCv_helper->setMask("mask.png");

    this->_init_retriever();

}

SceneRetriever::SceneRetriever(const string& voc,const string& scene_file)
{
    this->original_scene.loadFile(scene_file);
//    this->ploop_closing_manager_of_scene = new LoopClosingManager(voc);
    this->ploop_closing_manager_of_scene = shared_ptr<LoopClosingManager>(new LoopClosingManager(voc));

//    mpCv_helper = new cv_helper(360.0652, 363.2195, 406.6650, 256.2053, 39.9554);

    this->mpCv_helper = shared_ptr<cv_helper>(new cv_helper(360.0652, 363.2195, 406.6650, 256.2053, 39.9554));
    this->mpCv_helper->setMask("mask.png");

    this->_init_retriever();

    this->publishPoseHistory();
}


void SceneRetriever::_init_retriever()
{
    auto p2d = this->original_scene.getP2D();
    auto p3d = this->original_scene.getP3D();

    cout<<"SceneRetriever::init retriever start: "<<original_scene.getImageCount()<<endl;

    for(int frame_index = 0; frame_index < original_scene.getImageCount(); frame_index++)
    {

        struct FrameInfo* pfr = new struct FrameInfo;
	    
        pfr->keypoints = p2d[frame_index];
        pfr->descriptors = this->original_scene.getDespByIndex(frame_index);


        cout<<"pfr->keypoints size: "<<pfr->keypoints.size()<<endl;
        cout<<"pfr->descriptors size: "<<pfr->descriptors.size()<<endl;


        ptr_frameinfo frame_info(pfr);
        
        this->ploop_closing_manager_of_scene->addKeyFrame(frame_info);
    }

    cout<<"frameinfo_list.size()"<<this->ploop_closing_manager_of_scene->frameinfo_list.size()<<endl;
    cout<<"loaded database information: "<<this->ploop_closing_manager_of_scene->frame_db<<endl;
}


void SceneRetriever::setImageVecPath(vector<string>& imageVec, int left)
{
    if(left)
        this->mVecLeftImagePath = imageVec;

    if(!left)
        this->mVecRightImagePath = imageVec;
}


cv::Mat SceneRetriever::fetchImage(size_t index, int left)
{
    cout<<"fetchImage: "<<index<<", "<<left<<endl;
    cout<<"this->mVecLeftImagePath.size(): "<<this->mVecLeftImagePath.size()<<endl;

    if(left)
    {
        if(index >=0 && index < this->mVecLeftImagePath.size())
        {
            cout<<"cv::imread(this->mVecLeftImagePath[index]) size: "<<cv::imread(this->mVecLeftImagePath[index]).size()<<endl;
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
    cv::drawMatches(curImage, curKps, oldImage, oldKps, matches, output_image);

    if (!output_image.empty()) {
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
                                                 cv::Mat& Q_mat, cv::Mat& RT_mat_of_stereo_cam_output, bool& match_success,int* pMatchedIndexID_output)
{
    this->LoopClosureDebugIndex ++;

    //step<1> generate sparse pointcloud of image pair input and scene.
    if (image_left_rect.empty() || image_right_rect.empty())
    {
        cout<<"Left or Right image are empty, return."<<endl;
        match_success = false;
        return -1;
    }

    // apply mask to input image
    mpCv_helper->applyMask(image_right_rect);
    mpCv_helper->applyMask(image_left_rect);
    
//    cv::imshow("left image", image_left_rect);
//    cv::waitKey(5);

    this->mCurrentImage = image_left_rect;

    //<1>-(1) match left image with scene.
    std::vector<cv::DMatch> good_matches_output;
    ptr_frameinfo frameinfo_left = this->ploop_closing_manager_of_scene->extractFeature(image_left_rect);

    int loop_index= this->ploop_closing_manager_of_scene->detectLoopByKeyFrame(frameinfo_left, good_matches_output, true);

    cout<<"Loop Index: "<<loop_index<<endl;
    if(loop_index<0)
    {
        //frame match failed.
        match_success = false;
        return -1;
    }

    //NOTE display feature matches between current frame and detected old frame
    cout << "good_matches_output: " << good_matches_output.size() << endl;

    //step 1, fetch current frame camera points and desps

    cout<<"icp fetch current frame camera points and desps"<<endl;

    vector<cv::KeyPoint> current_kps_left;
    vector<cv::Point3f> current_camera_pts;
    cv::Mat current_frame_desps;
    if(!mpCv_helper->StereoImage2CamPoints(image_left_rect, image_right_rect, current_kps_left, current_camera_pts, current_frame_desps))
    {
        match_success = false;
        return -1;
    }

    cout<<"icp fetch old frame camera points and desps"<<endl;

    //step 2, fetch old frame camera points and desps
    cv::Mat old_image_left = this->fetchImage(loop_index, 1);
    cv::Mat old_image_right = this->fetchImage(loop_index, 0);

    // ------------------------------------------------------------------------------------------------------------------------------------
    // NOTE real time computed elements
//    vector<cv::KeyPoint> old_kps_left;
//    vector<cv::Point3f> old_camera_pts;
//    cv::Mat old_frame_desps;
//    cout<<"old_image_left size: "<<old_image_left.size()<<endl;
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
    cout<<"old_kps_left size: "<<old_kps_left.size()<<endl;
    cout<<"old_camera_pts_3d size: "<<old_camera_pts_3d.size()<<endl;
    cout<<"old_frame_desps size: "<<old_frame_desps.size()<<endl;

    // ------------------------------------------------------------------------------------------------------------------------------------
    /*
    vector<cv::KeyPoint> old_kps_left;
    vector<cv::Point3f> old_camera_pts;
    cv::Mat old_frame_desps;
    cout<<"old_image_left size: "<<old_image_left.size()<<endl;
    if(!mpCv_helper->StereoImage2CamPoints(old_image_left, old_image_right, old_kps_left, old_camera_pts, old_frame_desps))
    {
        match_success = false;
        return -1;
    }
    */

    //step 3, match current and old features
    vector<cv::DMatch> result_matches;
    mpCv_helper->match2Images(current_kps_left, current_frame_desps,
                              old_kps_left, old_frame_desps,
                              result_matches);

    cout<<"icp match current and old features"<<endl;

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
        matched_old_cam_pts.push_back(old_camera_pts[result_matches[i].trainIdx]);
    }

    cout<<"icp displayFeatureMatches"<<endl;

    //step 6, now that we have matched camera points we can conduct ICP, we can use either PCL method or opencv method
    this->displayFeatureMatches(image_left_rect, current_kps_left,
                                old_image_left, old_kps_left,
                                result_matches, loop_index);

    cout<<"icp GeneralICP and std::to_string(this->LoopClosureDebugIndex): "<<std::to_string(this->LoopClosureDebugIndex)<<endl;

    Eigen::Matrix4f result;
    float fitnesscore = mpCv_helper->GeneralICP(matched_current_cam_pts, matched_old_cam_pts, result);

    if(fitnesscore > 100)
    {
      //[MERGE_ERROR]What about match_success value????? LEAVE THIS COMPILE ERROR FOR YOU.
      match_success = false;//IS THIS CORRECT????
      return fitnesscore;
    }

    cout<<"get fitness score: "<<fitnesscore<<endl;
    //int result_size = mpCv_helper->GeneralICP(matched_current_cam_pts, matched_old_cam_pts, result);

    cout<<"icp given old T and relative loop closure T, get new T"<<endl;
    cout<<"Calculated transform matrix is: \n"<<result<<endl;

    //step 7, given old T and relative loop closure T, get new T
    cv::Mat result_relative_T;
    cv::eigen2cv(result, result_relative_T);
    cv::Mat result_R = result_relative_T.colRange(0,3).rowRange(0,3);
    cv::Mat result_t = result_relative_T.rowRange(0,3).col(3);


    //step 8, given old left image and current left image, compute relative R vec from rodrigues by essential mat
    if(current_kps_left.size() < 30 && old_kps_left.size() < 30)
    {
        match_success = false;//[MERGE_ERROR] is this correct??
        return -1;
    }
    cv::Mat essentialR = mpCv_helper->getRotationfromEssential(matched_current_kps, matched_old_kps);

    cv::Mat essentialRvec, RrelativeVec;
    cv::Rodrigues (essentialR, essentialRvec);
    cv::Rodrigues (result_R, RrelativeVec);

    cout<<"essentialRvec: "<<essentialRvec<<endl;
    cout<<"RrelativeVec: "<<RrelativeVec<<endl;

    float distanceR = mpCv_helper->Vec3Distance(essentialRvec, RrelativeVec);

    if(distanceR>0.3)
        return -1;


    //step 10, if R vector distance is small enough , consider as inlier and continue to the next step

    //fetch old frame R and t
    cv::Mat R = this->original_scene.getR(loop_index);
    cv::Mat t = this->original_scene.getT(loop_index);

    cout<<"R and t old are: "<<R<<", "<<t<<endl;

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

    cout<<"old T:\n"<<old_T<<endl;

    cv::Mat new_T;

    result_relative_T.convertTo(result_relative_T, CV_64F);

    new_T = old_T * result_relative_T;

    cv::Mat new_R = new_T.colRange(0,3).rowRange(0,3);
    cv::Mat new_t = new_T.rowRange(0,3).col(3);

    cout<<"new T:\n"<<new_T<<endl;

    if (fitnesscore < 0.5) //TODO:move this into a config.
    {
        this->mpCv_helper->publishPose(new_R, new_t, 0);
        RT_mat_of_stereo_cam_output = new_T;
        match_success = true;
        if(pMatchedIndexID_output != nullptr)
        {
            *pMatchedIndexID_output = loop_index;
        }
        return fitnesscore;
    }
    else
    {
        match_success = false;
        return fitnesscore;
    }
}



float SceneRetriever::retrieveSceneWithScaleFromMonoImage(cv::Mat image_left_rect, cv::Mat& cameraMatrix, cv::Mat& RT_mat_of_mono_cam_output, bool& match_success,int* pMatchedIndexID_output)
{
    if(this->original_scene.hasScale == false)
    {
        match_success = false;
        return -1;
    }

    // NOTE this is the working version of SHR
    this->LoopClosureDebugIndex ++;

    //step<1> generate sparse pointcloud of image pair input and scene.
    if (image_left_rect.empty())
    {
        cout<<"Left or Right image is empty, return."<<endl;
        match_success = false;
        return -1;
    }

    // apply mask to input image
    mpCv_helper->applyMask(image_left_rect);

//    cv::imshow("left image", image_left_rect);
//    cv::waitKey(5);

    this->mCurrentImage = image_left_rect;

    //<1>-(1) match left image with scene.
    std::vector<cv::DMatch> good_matches_output;
    ptr_frameinfo frameinfo_left = this->ploop_closing_manager_of_scene->extractFeature(image_left_rect);

    int loop_index= this->ploop_closing_manager_of_scene->detectLoopByKeyFrame(frameinfo_left, good_matches_output, true);

    cout<<"Loop Index: "<<loop_index<<endl;
    if(loop_index<0)
    {
        match_success = false;
        return -1;
    }

    //NOTE display feature matches between current frame and detected old frame
    cout << "good_matches_output: " << good_matches_output.size() << endl;

    if (good_matches_output.size() > 10) {
        this->displayFeatureMatches(loop_index, frameinfo_left, good_matches_output);
    }

    //fetch left and right image
    cv::Mat old_image_left = this->fetchImage(loop_index, 1);
    cv::Mat old_image_right = this->fetchImage(loop_index, 0);

    //fetch old frame R and t
    cv::Mat R = this->original_scene.getR(loop_index);
    cv::Mat t = this->original_scene.getT(loop_index);

    //initialize PnP result
    cv::Mat result_R, result_t;

    //conduct pnp
    if (old_image_right.empty() || old_image_left.empty() || R.empty() || t.empty())
    {
        cout<<"Image empty or Rt empty before solvePnP"<<endl;
        match_success = false;
        return -1;
    }
    else
    {
        int pnpResult = this->mpCv_helper->solvePnP(old_image_left,
                                                     old_image_right,
                                                     image_left_rect,
                                                     R, t,
                                                     result_R, result_t);

        cout<<"pnpresult: "<<pnpResult<<endl;
        cout<<"pnpresult: "<<bool(pnpResult)<<endl;
        if(pnpResult>0 && !result_R.empty() && !result_t.empty())
        {
            cout<<"pnp result are: \n: "<<result_R<<endl<<result_t<<endl;
            cout<<"solve pnp finished, publishing the result."<<endl;

            cv::Mat temp_t =  -result_R.t()*result_t;

            //this->mpCv_helper->publishPose(result_R, result_t, 0);
            this->mpCv_helper->publishPose(-result_R.t(), temp_t, 0);

            cout<<"solve pnp finished, publishing the result finished."<<endl;

            match_success = true;
            if(pMatchedIndexID_output!=nullptr)
            {
                *pMatchedIndexID_output = loop_index;
            }
            return pnpResult;
        }
        else
        {
            match_success = false;
            return -1;
        }

    }



    // NOTE this is the original version of WHY

//    ptr_frameinfo mono_image_info = this->ploop_closing_manager_of_scene->extractFeature(image_in_rect);
//    std::vector <cv::DMatch> good_matches;
//
//    int loop_index = this->ploop_closing_manager_of_scene->detectLoopByKeyFrame(mono_image_info,good_matches,false);
//
//    if (loop_index<0)
//    {
//        return -1;//Loop not found!
//    }
//
//    //query 3d points and do 2d-3d matching by PnP ransac.
//    std::vector<cv::Point3f> points3d;
//    std::vector<cv::Point2f> projectedPoints2d;
//
//    for(auto &match:good_matches)
//    {
//        auto pt1 = mono_image_info->keypoints[match.queryIdx].pt;//image point 2d.
//        projectedPoints2d.push_back(pt1);
//        auto pt2 = this->original_scene.getP3D().at(loop_index)[match.trainIdx];//scene point 3d.
//		    //image point 2d of scene: this->loop_closing_manager_of_scene.getFrameInfoById(loop_index)->keypoints[match.trainIdx].pt;
//        points3d.push_back(pt2);
//    }
//
//    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
//    cv::Mat rvec,tvec,inliers;
//
//    bool cv_solvepnpransac_result = cv::solvePnPRansac (points3d,//3d
//        projectedPoints2d,//2d
//        cameraMatrix,
//        distCoeffs,
//        rvec,
//        tvec,
//        //bool useExtrinsicGuess =
//        false,
//        //int iterationsCount =
//	100,
//        //float reprojectionError =
//	8.0,
//        //double  confidence =
//	0.99,
//        inliers,
//        //int flags =
//	cv::SOLVEPNP_ITERATIVE
//        );
//    //check inliers.
//    cv::Mat R,retMat;
//    if(cv_solvepnpransac_result )//&& inliers.size()>8)
//    {
//      cv::Rodrigues(rvec,R);//match success
//      retMat=cv::Mat::eye(4,4,CV_32F);
//      retMat.rowRange(cv::Range(0,3)).colRange(cv::Range(0,3)) = R;
//      retMat.colRange(3,1).rowRange(0,3) = tvec;
//      RT_mat_of_mono_cam_output = retMat;
//      return loop_index;
//    }
//    else
//    {
//        return -1;
//    }
//    //return.
    
}


void SceneRetriever::publishPoseHistory()
{
    assert(this->mpCv_helper != nullptr);

    vector<cv::Mat> vecT = this->original_scene.mVecT;

    for(auto& t: this->original_scene.mVecT)
    {
        if(t.empty())
            continue;

        cout<<"publishing pose: "<<t<<endl;

        cv::Mat R = cv::Mat::ones(3, 3, CV_32F);
        this->mpCv_helper->publishPose(R, t);
    }
}
