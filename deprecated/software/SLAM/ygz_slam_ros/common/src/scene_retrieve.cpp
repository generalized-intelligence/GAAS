#include "ygz/scene_retrieve.h"

/*SceneRetriever::SceneRetriever(Scene& original_scene_input)
{
    this->original_scene=original_scene;
    this->_init_retriever();
}*/

Scene::Scene()
{

    mpCv_helper = new cv_helper(360.0652, 363.2195, 406.6650, 256.2053, 39.9554);

    return;//TODO:fill in init functions.
}


int Scene::getImageCount()
{
    return this->vec_p2d.size();
}


void Scene::saveFile(const std::string &filename)
{
    
    std::ofstream ofs(filename);

    {
        boost::archive::text_oarchive oa(ofs);
        oa << *this;
    }
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

//     // lets do something with this vocabulary
//     cout << "Matching images against themselves (0 low, 1 high): " << endl;
//     BowVector v1, v2;
//     for(int i = 0; i < NIMAGES; i++)
//     {
//     voc.transform(features[i], v1);
//     for(int j = i-4; j < i+4; j++)
//     {
//     if(j<0)
//     {
//         continue;
//     }
//     if(j>=NIMAGES)
//     {
//         continue;
// 
//     }
//     voc.transform(features[i], v2);
// 
//     double score = voc.score(v1, v2);
//     cout << "Image " << i << " vs Image " << j << ": " << score << endl;
//     }
//     }

    // save the vocabulary to disk
    cout << endl << "Saving vocabulary..." << endl;
    voc.save("small_voc.yml.gz");
    cout << "Done" << endl;
    
}


void Scene::loadFile(const std::string &filename)
{

    std::ifstream ifs(filename);

    {
        boost::archive::text_iarchive ia(ifs);
        ia >> *this;
        cout << "Deserialization finished" << endl;
    }
}

void Scene::test()
{
    cout<<"---------------Current scene info---------------"<<endl;
    cout<<"mIndex: "<<mIndex<<endl;
    cout<<"hasScale: "<<hasScale<<endl;
    cout<<"vec_p2d size: "<<vec_p2d.size()<<endl;
    cout<<"vec_p3d size: "<<vec_p3d.size()<<endl;
    cout<<"point_desps size: "<<point_desps.size()<<endl;
    cout<<"vec_r size: "<<mVecR.size()<<endl;
    cout<<"vec_t size: "<<mVecT.size()<<endl;
    
    for(auto& m: this->mVecR)
    {
        cout<<"VecR is :"<<endl<<m<<endl;
    }
    
    for(auto& m: this->mVecT)
    {
        cout<<"VecT is :"<<endl<<m<<endl;
    }
    
    cout<<"---------------Current scene info---------------"<<endl;
}



SceneFrame Scene::generateSceneFrameFromStereoImage(cv::Mat imgl, cv::Mat imgr, cv::Mat RotationMat, cv::Mat TranslationMat, cv::Mat Q_mat)
{

    cout<<"generateSceneFrameFromStereoImage 1"<<endl;

    LoopClosingManager lcm("./voc/brief_k10L6.bin");

    std::vector<cv::KeyPoint> key_points2d_candidate;
    std::vector<cv::KeyPoint> key_points2d_final;
    std::vector<cv::Point3d> points3d;
    cv::Mat feature;

    ptr_frameinfo pleft_image_info = lcm.extractFeature(imgl);
    ptr_frameinfo pright_image_info = lcm.extractFeature(imgr);


    cout<<"pleft_image_info kps size: "<<pleft_image_info->keypoints.size()<<endl;
    cout<<"pright_image_info kps size: "<<pright_image_info->keypoints.size()<<endl;


    cv::Mat feature_l,feature_r;
    feature_l = pleft_image_info->descriptors;
    feature_r = pright_image_info->descriptors;

    std::vector<int> good_2dmatches_index;

    FlannBasedMatcher matcher = FlannBasedMatcher(makePtr<flann::LshIndexParams>(12,20,2));
    //FlannBasedMatcher matcher = FlannBasedMatcher();

    std::vector< DMatch > matches;
    matcher.match(feature_l, feature_r, matches);

    double max_dist = 0;
    double min_dist = 100;
    for( int i = 0; i < matches.size(); i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    std::vector< DMatch > good_matches;
    for( int i = 0; i < matches.size(); i++ )
    {
        if( matches[i].distance <= 2*min_dist && matches[i].distance< 10) // 3.0 too large;2.0 too large.
        {
            good_matches.push_back( matches[i]);
        }
    }

    std::vector<Point2f> lk_input_keypoints, lk_output_keypoints;
    for(size_t i = 0; i < good_matches.size(); i++)
    {
        lk_input_keypoints.push_back(pleft_image_info->keypoints[good_matches[i].queryIdx].pt);//will check if LKFlow exist.
        good_2dmatches_index.push_back(good_matches[i].queryIdx);
    }


    std::vector<unsigned char> PyrLKResults;
    std::vector<float> optflow_err;

    //Size winSize(31,31);
    //TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);

    cout<<"lk_input_keypoints.size(): "<<(lk_input_keypoints.size())<<endl;

    if (lk_input_keypoints.size()<=5)
    {
        SceneFrame failed;
        return failed;
    }

    cv::calcOpticalFlowPyrLK(imgl,
                             imgr,
                             lk_input_keypoints,
                             lk_output_keypoints,
                             PyrLKResults,
                             optflow_err);


    cout<<"lk_output_keypoints.size() == 0: "<<(lk_output_keypoints.size())<<endl;
    cout<<"PyrLKResults.size() == 0: "<<(PyrLKResults.size())<<endl;


    if (lk_input_keypoints.size() < 5 || lk_output_keypoints.size() < 5 || PyrLKResults.size() < 5)
    {
        cout<<"lk_input_keypoints.size() == 0: "<<(lk_input_keypoints.size() == 0)<<endl;
        cout<<"lk_output_keypoints.size() == 0: "<<(lk_output_keypoints.size() == 0)<<endl;
        cout<<"PyrLKResults.size() == 0: "<<(PyrLKResults.size() == 0)<<endl;
    }


    std::vector<Point2f> matched_points;
    std::vector<float> disparity_of_points;
    cv::Mat descriptors_reserved;


    for(int index = 0; index<lk_input_keypoints.size(); index++)
    {
        if(PyrLKResults[index] == 1)
        {
            matched_points.push_back(lk_input_keypoints[index]);
            disparity_of_points.push_back(lk_input_keypoints[index].x - lk_output_keypoints[index].x);

            //Mat desp_reserved = pleft_image_info->descriptors.colRange(good_2dmatches_index[index], good_2dmatches_index[index]+1).clone().reshape(0);
            Mat desp_reserved = pleft_image_info->descriptors.row(good_2dmatches_index[index]).clone().reshape(0);
            descriptors_reserved.push_back(desp_reserved);
        }
    }

    cout<<"Q_mat: "<<Q_mat<<endl;

    std::vector<cv::Point3f> points3f;

//    points3f = mpCv_helper->image2world(lk_input_keypoints, disparity_of_points, RotationMat, TranslationMat);

    points3f = mpCv_helper->image2world(lk_input_keypoints, disparity_of_points, RotationMat, TranslationMat);

    points3d = mpCv_helper->Points3f2Points3d(points3f);


//    cv::reprojectImageTo3D(disparity_of_points, points3f, Q_mat);
//
//
//    //do rotation and translation to points3d.
//    for(int i = 0;i<points3d.size();i++)
//    {
//        cv::Mat point3d_temp(points3d[i]);
//
//        Eigen::Matrix3f rotation;
//        Eigen::Vector3f translation;
//        Eigen::Vector3f point;
//        Eigen::Vector3f result;
//
//        cv::cv2eigen(RotationMat, rotation);
//        cv::cv2eigen(TranslationMat, translation);
//        cv::cv2eigen(point3d_temp, point);
//
//        result = rotation * point + translation;
//
//        //cv::Mat transformed = RotationMat*point3d_temp + TranslationMat;
//
//        cv::Mat transformed;
//        cv::eigen2cv(result, transformed);
//
//        Point3d output;
//        output.x = transformed.at<float>(0);
//        output.y = transformed.at<float>(1);
//        output.z = transformed.at<float>(2);
//        points3d[i] = output;//do transform in mat form.
//    }

    cv::KeyPoint::convert(matched_points, key_points2d_final);

    cout<<"generate scene, points3d size: "<<points3d.size()<<endl;




    //TODO for debug purposes
    key_points2d_final = pleft_image_info->keypoints;
    descriptors_reserved = pleft_image_info->descriptors;

    return std::make_tuple(key_points2d_final, points3d, descriptors_reserved, RotationMat, TranslationMat);

}




/////////////////////////////////////////////////////////////////////////////////////////////////


SceneRetriever::SceneRetriever()
{

}

SceneRetriever::SceneRetriever(const string& voc,const string& scene_file)
{
    this->original_scene.loadFile(scene_file);
    this->ploop_closing_manager_of_scene = new LoopClosingManager(voc);
    this->_init_retriever();
}

void SceneRetriever::_init_retriever()
{
    auto p2d = this->original_scene.getP2D();
    auto p3d = this->original_scene.getP3D();

    for(int frame_index = 0; frame_index<original_scene.getImageCount(); frame_index++)
    {
        ptr_frameinfo pfr = shared_ptr<FrameInfo>(new FrameInfo());
	    pfr->keypoints = p2d[frame_index];
	    pfr->descriptors = this->original_scene.getDespByIndex(frame_index);
        
        this->ploop_closing_manager_of_scene->addKeyFrame(pfr);
    }
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
  
    //<1>-(1) match left image with scene.
    std::vector<DMatch> good_matches_output;
    ptr_frameinfo frameinfo_left = this->ploop_closing_manager_of_scene->extractFeature(image_left_rect);
    int loop_index= this->ploop_closing_manager_of_scene->detectLoopByKeyFrame(frameinfo_left, good_matches_output, false);
    if(loop_index<0)
    {
        //frame match failed.
        cout<<"frame match failed!"<<endl;
        return -1;
    }
    
    //<1>-(2) calc left image point 3d position.LoopClosingManager
    //ptr_frameinfo frameinfo_left = this->ploop_closing_manager_of_scene->extractFeature(image_left_rect);
    std::vector<Point2f> InputKeypoints;
    std::vector<Point2f> PyrLKmatched_points;
    //for(int index = 0;index<frameinfo_left->keypoints.size();index++)
    for(int index = 0;index<good_matches_output.size();index++)// iterate matches.
    {
        int kp_index = good_matches_output[index].queryIdx;
        InputKeypoints.push_back(frameinfo_left->keypoints[kp_index].pt);//only reserve matched points.
    }
    
    std::vector<unsigned char> PyrLKResults;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK( image_left_rect,
		image_right_rect,
		InputKeypoints,
		PyrLKmatched_points,
		PyrLKResults,
		err
	);
    
    std::vector<Point2f> matched_points;
    std::vector<float> disparity_of_points;
    for(int index = 0; index<frameinfo_left->keypoints.size(); index++)
    {
        if(PyrLKResults[index] == 1)
        {
            matched_points.push_back(InputKeypoints[index]);
            disparity_of_points.push_back(PyrLKmatched_points[index].x-InputKeypoints[index].x);
        }
    }
    std::vector<Point3f> points_3d;
    cv::reprojectImageTo3D(disparity_of_points, points_3d, Q_mat);

    
    //step<2> match 2 clouds.
      //method<1>
      /*
        GICP = pcl::GeneralizedIterativeClosestPoint< PointSource, PointTarget >::estimateRigidTransformationBFGS 	( 	const PointCloudSource &  	cloud_src, //3d to 3d.
		const std::vector< int > &  	indices_src, 
		const PointCloudTarget &  	cloud_tgt,
		const std::vector< int > &  	indices_tgt,
		Eigen::Matrix4f &  	transformation_matrix 
	) 	*/
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
    std::vector <DMatch> good_matches;
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
      retMat.rowRange(Range(0,3)).colRange(Range(0,3)) = R;
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
