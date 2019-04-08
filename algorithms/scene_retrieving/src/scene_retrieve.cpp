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
        cout << "Deserialization finished" << endl;
    }

    this->test();

//    this->removeEmptyElement();
//    this->test();

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
    if(left)
    {
        if(index >0 && index < this->mVecLeftImagePath.size())
        {
            return cv::imread(this->mVecLeftImagePath[index]);
        }
    }
    else
    {
        if(index >0 && index < this->mVecRightImagePath.size())
        {
            return cv::imread(this->mVecRightImagePath[index]);
        }
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



int SceneRetriever::retrieveSceneFromStereoImage(cv::Mat image_left_rect, cv::Mat image_right_rect, cv::Mat& Q_mat, cv::Mat& RT_mat_of_stereo_cam_output, bool& match_success)
{
    this->LoopClosureDebugIndex ++;

    //step<1> generate sparse pointcloud of image pair input and scene.
    if (image_left_rect.empty() && image_right_rect.empty())
    {
        cout<<"Left or Right image are empty, return."<<endl;
        return -1;
    }

    // apply mask to input image
    mpCv_helper->applyMask(image_right_rect);
    mpCv_helper->applyMask(image_left_rect);
    
    cv::imshow("left image", image_left_rect);
    //cv::waitKey(5);

    this->mCurrentImage = image_left_rect;

    //<1>-(1) match left image with scene.
    std::vector<cv::DMatch> good_matches_output;
    ptr_frameinfo frameinfo_left = this->ploop_closing_manager_of_scene->extractFeature(image_left_rect);

    int loop_index= this->ploop_closing_manager_of_scene->detectLoopByKeyFrame(frameinfo_left, good_matches_output, true);

    cout<<"Loop Index: "<<loop_index<<endl;
    if(loop_index<0)
    {
        //frame match failed.
        return -1;
    }

    //NOTE display feature matches between current frame and detected old frame
    cout << "good_matches_output: " << good_matches_output.size() << endl;
    if (good_matches_output.size() > 5) {
        this->displayFeatureMatches(loop_index, frameinfo_left, good_matches_output);
    }


    // method 1

//    solvePnP(cv::Mat& old_image_left,
//             cv::Mat& old_image_right,
//             cv::Mat& cur_image_left,
//             cv::Mat& cur_image_right,
//             cv::Mat R, cv::Mat t, cv::Mat Transformation)

    //fetch left and right image
    cv::Mat old_image_left = this->fetchImage(loop_index, 1);
    cv::Mat old_image_right = this->fetchImage(loop_index, 0);

    //fetch old frame R and t
    cv::Mat R = this->original_scene.getR(loop_index);
    cv::Mat t = this->original_scene.getT(loop_index);

    //initialize PnP result
    cv::Mat result_R, result_t;

    //conduct pnp
    bool pnpResult = this->mpCv_helper->solvePnP(old_image_left,
                                old_image_right,
                                image_left_rect,
                                image_right_rect,
                                R, t,
                                result_R, result_t);



    if(pnpResult)
    {
        cout<<"pnp result are: \n: "<<result_R<<endl<<result_t<<endl;
        cout<<"solve pnp finished, publishing the result."<<endl;

        cv::Mat temp_t =  -result_R.t()*result_t;

        //this->mpCv_helper->publishPose(result_R, result_t, 0);
        this->mpCv_helper->publishPose(-result_R.t(), temp_t, 0);

        cout<<"solve pnp finished, publishing the result finished."<<endl;
    }







    // method 2

    if(0)
    {

        cout << "good_matches_output: " << good_matches_output.size() << endl;

        if (good_matches_output.size() > 5) {
            this->displayFeatureMatches(loop_index, frameinfo_left, good_matches_output);
        }


        //NOTE continue to the next step if there are more than 20 matched points



        //<1>-(2) calc left image point 3d position.LoopClosingManager
        //ptr_frameinfo frameinfo_left = this->ploop_closing_manager_of_scene->extractFeature(image_left_rect);
        std::vector <cv::Point2f> InputKeypoints;
        std::vector <cv::Point2f> PyrLKmatched_points;

        for (int index = 0; index < good_matches_output.size(); index++)// iterate matches.
        {
            int kp_index = good_matches_output[index].queryIdx;
            InputKeypoints.push_back(frameinfo_left->keypoints[kp_index].pt);//only reserve matched points.
        }


        for (auto &p: InputKeypoints) {
            cout << "Recovered 2D point: " << p << endl;
        }


        std::vector<unsigned char> PyrLKResults;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(image_left_rect,
                                 image_right_rect,
                                 InputKeypoints,
                                 PyrLKmatched_points,
                                 PyrLKResults,
                                 err
        );


        std::vector <cv::Point2f> matched_points;
        std::vector<float> disparity_of_points;

        for (int index = 0; index < InputKeypoints.size(); index++) {
            if (PyrLKResults[index] == 1) {
                matched_points.push_back(InputKeypoints[index]);

                //disparity_of_points.push_back(PyrLKmatched_points[index].x - InputKeypoints[index].x);
                disparity_of_points.push_back(InputKeypoints[index].x - PyrLKmatched_points[index].x);
            }
        }


        for (auto &p: PyrLKmatched_points) {
            cout << "LKflow detected pt: " << p << endl;
        }


        for (auto &p: disparity_of_points) {
            cout << "Recovered pts disp: " << p << endl;
        }

        cout << "cv helper 1" << endl;

        vector <cv::Point3f> CamPoints;

        {
            CamPoints = mpCv_helper->image2cam(InputKeypoints, disparity_of_points);
        }


        cout << "cv helper 1.5" << endl;

        for (auto &p: CamPoints) {
            cout << "Recovered Cam point: " << p << endl;
        }

        cout << "cv helper 2" << endl;

        //NOTE check:
        //https://github.com/PointCloudLibrary/pcl/blob/master/test/registration/test_registration.cpp
        //for more information about the usage

        vector <cv::Point3f> frameOldMapPoints = mpCv_helper->Points3d2Points3f(
                (this->original_scene).fetchFrameMapPoints(loop_index));

        cout << "cv helper 3" << endl;

        if (CamPoints.size() > 20 && frameOldMapPoints.size() > 20) {
            cout << "Start general ICP!" << endl;
            Matrix4f transformation = mpCv_helper->GeneralICP(CamPoints, frameOldMapPoints);
            cout << "General ICP result is: \n" << transformation << endl;
        } else {
            cout << "GeneralICP requires at least 20 pairs of points, quit." << endl;
            cout << "Current size is: " << CamPoints.size() << ", " << frameOldMapPoints.size() << endl;
        }

        cout << "cv helper 4" << endl;

    }
      //method<2>
      /*
       * SCIA = pcl::SampleConsensusInitialAlignment< PointSource, PointTarget, FeatureT >
       * SCIA = computeTransformation 	( 	PointCloudSource &  	output,		//useless.
		const Eigen::Matrix4f &  	guess 
	) 	
       */



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
        this->mpCv_helper->publishPose(cv::Mat(), t);
    }
}




int SceneRetriever::retrieveSceneWithScaleFromMonoImage(const cv::Mat image_in_rect,const cv::Mat& cameraMatrix, cv::Mat& RT_mat_of_mono_cam_output, bool& match_success)
{
    if(this->original_scene.hasScale == false)
    {
        match_success = false;
        return -1;
    }
    ptr_frameinfo mono_image_info = this->ploop_closing_manager_of_scene->extractFeature(image_in_rect);
    std::vector <cv::DMatch> good_matches;
    int loop_index = this->ploop_closing_manager_of_scene->detectLoopByKeyFrame(mono_image_info,good_matches,false);
    if (loop_index<0)
    {
        return -1;//Loop not found!
    }
    //query 3d points and do 2d-3d matching by PnP ransac.
    
    std::vector<cv::Point3f> points3d;
    std::vector<cv::Point2f> projectedPoints2d;
    
    for(auto &match:good_matches)
    {
        auto pt1 = mono_image_info->keypoints[match.queryIdx].pt;//image point 2d.
        projectedPoints2d.push_back(pt1);
        auto pt2 = this->original_scene.getP3D().at(loop_index)[match.trainIdx];//scene point 3d.
		    //image point 2d of scene: this->loop_closing_manager_of_scene.getFrameInfoById(loop_index)->keypoints[match.trainIdx].pt;
        points3d.push_back(pt2);
    }
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat rvec,tvec,inliers;
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
    cv::Mat R,retMat;
    if(cv_solvepnpransac_result )//&& inliers.size()>8)
    {
      cv::Rodrigues(rvec,R);//match success
      retMat=cv::Mat::eye(4,4,CV_32F);
      retMat.rowRange(cv::Range(0,3)).colRange(cv::Range(0,3)) = R;
      retMat.colRange(3,1).rowRange(0,3) = tvec;
      RT_mat_of_mono_cam_output = retMat;
      return loop_index;
    }
    else
    {
        return -1;
    }
    //return.
    
}
