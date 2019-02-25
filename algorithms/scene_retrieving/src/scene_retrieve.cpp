

std::pair<std::vector<std::vector<DMatch>>,std::vector<int>> SceneRetriever::matchImageWithScene2D(const cv::Mat image);
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
	
}
int SceneRetriever::retrieveSceneFromStereoImage(const cv::Mat image_left_rect, const cv::Mat image_right_rect, double camera_bf, cv::Mat& RT_mat_of_stereo_cam_output, bool& match_success)
{
    //step<1> generate sparse pointcloud of image pair input and scene.
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



int SceneRetriever::retrieveSceneWithScaleFromMonoImage(const cv::Mat image_in, cv::Mat& RT_mat_of_mono_cam_output, bool& match_success)
{
    if(this->original_scene.hasScale == false)
    {
        match_success = false;
        return -1;
    }
    
    bool cv_solvepnpransac_result = cv::solvePnPRansac (InputArray objectPoints,//3d
        InputArray imagePoints,//2d
        InputArray cameraMatrix,
        InputArray distCoeffs,
        OutputArray rvec,
        OutputArray tvec,
        bool useExtrinsicGuess = false,
        int iterationsCount = 100,
        float reprojectionError = 8.0,
        double  confidence = 0.99,
        OutputArray  inliers = noArray(),
        int flags = SOLVEPNP_ITERATIVE 
        )
    //check inliers.
    //return.
    
}
