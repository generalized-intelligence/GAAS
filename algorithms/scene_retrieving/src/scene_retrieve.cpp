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

    test();

    removeEmptyElement();

    test();
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
//    this->original_scene.test(true);

    this->ploop_closing_manager_of_scene = new LoopClosingManager(voc);
    
    this->_init_retriever();
}


void SceneRetriever::_init_retriever()
{
    auto p2d = this->original_scene.getP2D();
    auto p3d = this->original_scene.getP3D();

    for(int frame_index = 0; frame_index < original_scene.getImageCount(); frame_index++)
    {   
        struct FrameInfo* pfr = new struct FrameInfo;
	    
        pfr->keypoints = p2d[frame_index];
	    pfr->descriptors = this->original_scene.getDespByIndex(frame_index);
        
        ptr_frameinfo frame_info(pfr);
        
        this->ploop_closing_manager_of_scene->addKeyFrame(frame_info);
    
    }

    mpCv_helper = new cv_helper(360.0652897865692, 363.2195731683743, 406.6650580307593, 256.20533579714373, 39.9554);

    cout<<"loaded database information: "<<this->ploop_closing_manager_of_scene->frame_db<<endl;
}


/*std::pair<std::vector<std::vector<DMatch> >,std::vector<int>> SceneRetriever::matchImageWithScene2D(const cv::Mat image);
{
    //step<1> extract feature.
    for (int i=0;i<this->original_scene.getImageCount();i++)
    {
      cv::Mat& desp = this->original_scene.getDespByIndex(i);
      ...//generate "info".
      this->loop_closing_manager_of_scene.addKeyFrame(ptr_frameinfo info);
    }
    
    //step<2> find loop with scene by loop closing algorithms.
    this->loop_closing_manager_of_scene.queryKeyFrames(...);
    //step<3> do match.return 2d matching.
    return ...
	
}*/


int SceneRetriever::retrieveSceneFromStereoImage(const cv::Mat image_left_rect, const cv::Mat image_right_rect, const cv::Mat& Q_mat, cv::Mat& RT_mat_of_stereo_cam_output, bool& match_success)
{

    //step<1> generate sparse pointcloud of image pair input and scene.
    if (image_left_rect.empty() && image_right_rect.empty())
    {
        cout<<"Left or Right image are empty, return."<<endl;
        return -1;
    }
    
    cv::imshow("left image", image_left_rect);
    cv::waitKey(10);
    
    //<1>-(1) match left image with scene.
    std::vector<cv::DMatch> good_matches_output;
    ptr_frameinfo frameinfo_left = this->ploop_closing_manager_of_scene->extractFeature(image_left_rect);
    
    int loop_index= this->ploop_closing_manager_of_scene->detectLoopByKeyFrame(frameinfo_left, good_matches_output, true);
    if(loop_index<0)
    {
        //frame match failed.
        return -1;
    }

    cout<<"good_matches_output: "<<good_matches_output.size()<<endl;
    
    
    //<1>-(2) calc left image point 3d position.LoopClosingManager
    //ptr_frameinfo frameinfo_left = this->ploop_closing_manager_of_scene->extractFeature(image_left_rect);
    std::vector<cv::Point2f> InputKeypoints;
    std::vector<cv::Point2f> PyrLKmatched_points;

    for(int index = 0;index<good_matches_output.size();index++)// iterate matches.
    {
        int kp_index = good_matches_output[index].queryIdx;
        InputKeypoints.push_back(frameinfo_left->keypoints[kp_index].pt);//only reserve matched points.
    }
    
    
    for (auto& p: InputKeypoints)
    {
        cout<<"Recovered 2D point: "<<p<<endl;   
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
    
    
    std::vector<cv::Point2f> matched_points;
    std::vector<float> disparity_of_points;

    for(int index = 0; index < InputKeypoints.size(); index++)
    {
        if(PyrLKResults[index] == 1)
        {
            matched_points.push_back(InputKeypoints[index]);

            //disparity_of_points.push_back(PyrLKmatched_points[index].x - InputKeypoints[index].x);
            disparity_of_points.push_back(InputKeypoints[index].x - PyrLKmatched_points[index].x);
        }
    }

    
    for (auto& p: PyrLKmatched_points)
    {
        cout<<"LKflow detected pt: "<<p<<endl;   
    }
    
    
    for (auto& p: disparity_of_points)
    {
        cout<<"Recovered pts disp: "<<p<<endl;   
    }


    vector<cv::Point3f> CamPoints = mpCv_helper->image2cam(InputKeypoints, disparity_of_points);

    for (auto& p: CamPoints)
    {
        cout<<"Recovered Cam point: "<<p<<endl;
    }


    //NOTE check:
    //https://github.com/PointCloudLibrary/pcl/blob/master/test/registration/test_registration.cpp
    //for more information about the usage

    vector<cv::Point3f> frameOldMapPoints = mpCv_helper->Points3d2Points3f( (this->original_scene).fetchFrameMapPoints(loop_index) );

    if(CamPoints.size()>20 && frameOldMapPoints.size()>20)
    {
        cout<<"Start general ICP!"<<endl;
        Matrix4f transformation = mpCv_helper->GeneralICP(CamPoints, frameOldMapPoints);
        cout<<"General ICP result is: \n"<<transformation<<endl;
    }
    else
    {
        cout<<"GeneralICP requires at least 20 pairs of points, quit."<<endl;
        cout<<"Current size is: "<<CamPoints.size()<<", "<<frameOldMapPoints.size()<<endl;
    }






      //method<2>
      /*
       * SCIA = pcl::SampleConsensusInitialAlignment< PointSource, PointTarget, FeatureT >
       * SCIA = computeTransformation 	( 	PointCloudSource &  	output,		//useless.
		const Eigen::Matrix4f &  	guess 
	) 	
       */
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
