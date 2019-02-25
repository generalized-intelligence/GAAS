#include <ygz/LoopClosing.h>


namespace ygz {
    
    
    LoopClosing::LoopClosing(string voc_path)
    {
        //mVoc = new BriefVocabulary(voc_path);
        //mDB.setVocabulary(*mVoc, false, 0);
        cout<<"Loading Vocabulary finished!"<<endl;
    }
    
    
    //query the frame and add the frame to database
    int LoopClosing::DetectLoop(shared_ptr<Frame> pFrame, int frame_idx)
    {
        ;
    }
/*
    int LoopClosing::DetectLoop(shared_ptr<Frame> pFrame, int frame_idx)
    {
        // put image into image_pool; for visualization
        cv::Mat compressed_image;
        cv::Mat loop_result;
        

        int feature_num = pFrame->mFeaturesLeft.size();
        cv::resize(pFrame->mImLeft, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10),
                CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_idx] = compressed_image;
        
        //NOTE print loopclosing image pool size
        cout<<"LoopClosing image pool:"<<image_pool.size()<<endl;
        
        QueryResults QueryResult;
        
        //1st, query current frame and add current frame to database
        
        //Queries the database with some features and return QueryResult
        //int(4) means number of result to return, which is a vector, 4 means there would return 4 results
        //the last para means the frame distance that would be reuturned to this frame
        //mDB.query(pFrame->brief_descriptors, QueryResult, 4, frame_idx-50);
        
        
        //queried results from high to low and displaying the top 4 scores!!!!!!!!!
        cout<<"QueryResult size: "<<QueryResult.size()<<endl;
        
        if(QueryResult.size() > 0)
        {
            cout<<"queried scores size: "<<QueryResult.size()<<endl;
            cout<<"Current frame queried scores are: "<<QueryResult[0].Score<<endl;
        }

        
        // Ads an entry to the database and returns its index
        //mDB.add(pFrame->brief_descriptors);
        cout<<"added frame to database!"<<pFrame->IsKeyFrame()<<endl;
        
        
        //find loop is set to false as default
        bool find_loop = false;
        
        bool save_image = true;
        bool DEBUG_IMAGE = true;
        if (DEBUG_IMAGE)
        {
            loop_result = compressed_image.clone();
            if (QueryResult.size() > 0)
                putText(loop_result, "neighbour score:" + to_string(QueryResult[0].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            
                putText(loop_result, "raw image:", cv::Point2f(270, 10), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            
            cv::imwrite("/home/gishr/software/px4/slam_dev/x86_slam/temp_img/keyframe_neighbour/" + std::to_string(frame_idx) + ".png", loop_result);
        }
    
    
        //if there are at least one result and the highest is greater than 0.055
        if (QueryResult.size() >= 1 && QueryResult[0].Score > 0.05)
        {   
            cout<<"query result size bigger than 0."<<endl;
            //iterate through all the result
            for (unsigned int i = 1; i < QueryResult.size(); i++)
            {
                //skipping QueryResult[0]
                if (QueryResult[i].Score > 0.015)
                {          
                    find_loop = true;
                    
                    int tmp_index = QueryResult[i].Id;
                    if (DEBUG_IMAGE)
                    {
                        auto it = image_pool.find(tmp_index);
                        cv::Mat tmp_image = (it->second).clone();
                        putText(tmp_image, "loop score:" + to_string(QueryResult[i].Score),
                                cv::Point2f(10, 50),CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
                        cv::hconcat(loop_result, tmp_image, loop_result);
                        cv::imwrite("/home/gishr/software/px4/slam_dev/x86_slam/temp_img/keyframe_loopclosure/"+ std::to_string(frame_idx) + ".png", loop_result);
                    }
                }
            }
        }
        
        
        //if founded loop and current frame idx is greater than 50
        if (find_loop && frame_idx > 50)
        {
            int min_index = -1;
            for (unsigned int i = 0; i < QueryResult.size(); i++)
            {
                //zhe tama shenme shadiao luoji
                if (min_index == -1 || (QueryResult[i].Id < min_index && QueryResult[i].Score > 0.015))
                {
                    min_index = QueryResult[i].Id; 
                }
            }
            
            // return the frame index of loop closure
            return min_index;
            
        }
        else
            return -1;
        
    }
*/
    //given frame index, retrieve correspoding frame
    shared_ptr<Frame> LoopClosing::getFrame(size_t index)
    {
        list<shared_ptr<Frame> >::iterator it = mFrameList.begin();
        for (; it != mFrameList.end(); it++)   
        {
            if((*it)->index == index)
                break;
        }
        if (it != mFrameList.end())
            return *it;
        else
            return NULL;
    }
    
    
    /*
    void LoopClosing::optimized4DoF()
    {
        cout<<"Loop Clsoing:: optimize 4dof initialized!"<<endl;
        
        while(true)
        {
            int cur_index = -1;
            int first_looped_index = -1;
            m_optimize_buf.lock();
            while(!optimize_buf.empty())
            {
                //frame index in the buf
                cur_index = optimize_buf.front();
                first_looped_index = earliest_loop_index;
                optimize_buf.pop();
            }
            m_optimize_buf.unlock();
            
            //if current_idx remain the same
            if(cur_index != -1)
            {
                cout<<"optimize pose graph"<<endl;
                m_framelist.lock();
                shared_ptr<Frame> curFrame = getFrame(cur_index);
                
                // we have the current index of the frame
                int max_length = cur_index + 1;
                
                //initialize translation array
                double t_array[max_length][3];
                
                //initialize Quaternion array
                Quaterniond q_array[max_length];
                
                //initialize euler array
                double euler_array[max_length];
                
                //initialize sequence array
                double sequence_array[max_length];
                
                //------------------------ceres---------------------------
                ceres::Problem problem;
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
                //options.minimizer_progress_to_stdout = true;
                //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
                options.max_num_iterations = 5;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(0.1);
                //loss_function = new ceres::CauchyLoss(1.0);
                ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();
                
                list<shared_ptr<Frame> >::iterator it;
                int i = 0;
                for (it = mFrameList.begin(); it != mFrameList.end(); it++)
                {
                    if ((*it)->index < first_looped_index)
                        continue;
                    (*it)->local_index = i;
                    
                    Quaterniond tmp_q;
                    Matrix3d tmp_r;
                    Vector3d tmp_t;
                    
                    tmp_t = (*it)->Rwb();
                    tmp_r = (*it)->Twb();
                    tmp_q = tmp_r;
                    
                    t_array[i][0] = tmp_t(0);
                    t_array[i][1] = tmp_t(1);
                    t_array[i][2] = tmp_t(2);
                    q_array[i] = tmp_q;
                    
                    Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
                    euler_array[i][0] = euler_angle.x();
                    euler_array[i][1] = euler_angle.y();
                    euler_array[i][2] = euler_angle.z();
                    
                    sequence_array[i] = (*it)->sequence;
                    
                    problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
                    problem.AddParameterBlock(t_array[i], 3);
                    
                    if ((*it)->index == first_looped_index || (*it)->sequence == 0)
                    {
                        problem.SetParameterBlockConstant(euler_array[i]);
                        problem.SetParameterBlockConstant(t_array[i]);
                    }
                     
                     
                    //Dont know what it is
                    for (int j = 1; j < 5; j++)
                    {
                        if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                        {
                            Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                            Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                            relative_t = q_array[i-j].inverse() * relative_t;
                            double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                            ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                        relative_yaw, euler_conncected.y(), euler_conncected.z());
                            problem.AddResidualBlock(cost_function, NULL, euler_array[i-j], 
                                                    t_array[i-j], 
                                                    euler_array[i], 
                                                    t_array[i]);
                        }
                    }
                    
                    //add loop edge
                    if((*it)->has_loop)
                    {
                        assert((*it)->loop_index >= first_looped_index);
                        int connected_index = getFrame((*it)->loop_index)->local_index;
                        Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
                        Vector3d relative_t;
                        relative_t = (*it)->getLoopRelativeT();
                        double relative_yaw = (*it)->getLoopRelativeYaw();
                        
                        ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                relative_yaw, euler_conncected.y(), euler_conncected.z());
                        problem.AddResidualBlock(cost_function, loss_function, euler_array[connected_index], 
                                                                    t_array[connected_index], 
                                                                    euler_array[i], 
                                                                    t_array[i]);
                        
                    }
                    
                    if ((*it)->index == cur_index)
                        break;
                    i++;
                }
                m_keyframelist.unlock();
                
                ceres::Solve(options, &problem, &summary);
                std::cout << summary.BriefReport() << "\n";
                
                m_keyframelist.lock();
                i = 0;
                for (it = mFrameList.begin(); it != mFrameList.end(); it++)
                {
                    if ((*it)->index < first_looped_index)
                        continue;
                    Quaterniond tmp_q;
                    tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
                    Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                    Matrix3d tmp_r = tmp_q.toRotationMatrix();
                    (*it)-> SetPose(tmp_t, tmp_r);

                    if ((*it)->index == cur_index)
                        break;
                    i++;
                }
                
                Vector3d cur_t, vio_t;
                Matrix3d cur_r, vio_r;
                
                cur_t = curFrame->Twb;
                cur_r = curFrame->Rwb;
                
                //vio_T_w_i here is the relative translation between two consecutive frames in the window
                //and once loopclosure is finished it should change the these values in the window
//                 m_drift.lock();
//                 yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
//                 r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
//                 t_drift = cur_t - r_drift * vio_t;
//                 m_drift.unlock();
//                 //cout << "t_drift " << t_drift.transpose() << endl;
//                 //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;
//                 //cout << "yaw drift " << yaw_drift << endl;
// 
//                 it++;
//                 for (; it != keyframelist.end(); it++)
//                 {
//                     Vector3d P;
//                     Matrix3d R;
//                     (*it)->getVioPose(P, R);
//                     P = r_drift * P + t_drift;
//                     R = r_drift * R;
//                     (*it)->updatePose(P, R);
//                 }
//                 m_keyframelist.unlock();
//                 updatePath();
            
            }
            
        }
        
        
    }*/
    
    
    
    void LoopClosing::addKeyFrame(shared_ptr<Frame> pFrame, bool detect_loop)
    {
        Vector3d P_cur = pFrame->Twb();
        Matrix3d R_cur = pFrame->Rwb();
        
        pFrame->index = global_index;
        global_index ++;
        
        int loop_index = -1;
        
        if (detect_loop)
        {   
            loop_index = DetectLoop(pFrame, pFrame->index);
        }
        else
        {
            //mDB.add(pFrame->brief_descriptors);
        }
        
        
        //-------------------------------------------------------------------------------------
        if(loop_index != -1)
        {
            
            cout<<"LoopClosing::addKeyFrame, loopclosure detected! loop index is : "<<loop_index<<endl;
            
            shared_ptr<Frame> old_kf = getFrame(loop_index);
            
            // WHAT THE FUCK THAT'S A LOT OF WORK TO DO
            if (pFrame->findConnection(old_kf))
            {
//                 if (earliest_loop_index > loop_index || earliest_loop_index == -1)
//                 {
//                     earliest_loop_index = loop_index;
//                 }
//                 
//                 // R and T in world frame
//                 Vector3d w_P_old = old_kf->Twb();
//                 Matrix3d w_R_old = old_kf->Rwb();
//                 
//                 // R and T of current frame
//                 Vector3d vio_P_cur = pFrame->Twb();
//                 Matrix3d vio_R_cur = pFrame->Rwb();
//                 
//                 Vector3d relative_t;
//                 Quaterniond relative_q;
//                 
//                 relative_t = pFrame->getLoopRelativeT();
//                 relative_q = (pFrame->getLoopRelativeQ()).toRotationMatrix();
//                 
//                 Vector3d w_P_cur = w_R_old * relative_t + w_P_old;
//                 Matrix3d w_R_cur = w_R_old * relative_q;
//                 
//                 //define r t and yaw
//                 double shift_yaw;
//                 shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
//                 shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
//                 shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur; 
//                 
//                 if(old_kf->sequence != cur->pFrame && sequence_loop[cur_kf->sequence] == 0)
//                 {
//                     w_r_vio = shift_r;
//                     w_t_vio = shift_t;
//                     vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
//                     vio_R_cur = w_r_vio *  vio_R_cur;
//                     pFrame->SetPose(vio_R_cur, vio_P_cur);
//                     
//                     //list<shared_ptr<Frame> > mFrameList
//                     list<shared_ptr<Frame> >::iterator it = mFrameList.begin();
//                     for (; it != mFrameList.end(); it++)   
//                     {
//                         if((*it)->sequence == cur_kf->sequence)
//                         {
//                             Vector3d vio_P_cur;
//                             Matrix3d vio_R_cur;
//                             vio_P_cur = (*it)->Twb;
//                             vio_R_cur = (*it)->Rwb;
//                             
//                             //NOTE w_r_vio = shift_r && w_t_vio = shift_t;
//                             vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
//                             vio_R_cur = w_r_vio *  vio_R_cur;
//                             (*it)->SetPose(vio_R_cur, vio_P_cur);
//                         }
//                         sequence_loop[cur_kf->sequence] = 1;
//                     }
//                     m_optimize_buf.lock();
//                     optimize_buf.push(cur_kf->index);
//                     m_optimize_buf.unlock();
//                 }
//                 
//                 //TODO finish this part
// //                 m_keyframelist.lock();
// //                 Vector3d P;
// //                 Matrix3d R;
// //                 cur_kf->getVioPose(P, R);
// //                 P = r_drift * P + t_drift;
// //                 R = r_drift * R;
// //                 cur_kf->updatePose(P, R);
// //                 Quaterniond Q{R};
// //                 geometry_msgs::PoseStamped pose_stamped;
// //                 pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
// //                 pose_stamped.header.frame_id = "world";
// //                 pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
// //                 pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
// //                 pose_stamped.pose.position.z = P.z();
// //                 pose_stamped.pose.orientation.x = Q.x();
// //                 pose_stamped.pose.orientation.y = Q.y();
// //                 pose_stamped.pose.orientation.z = Q.z();
// //                 pose_stamped.pose.orientation.w = Q.w();
// //                 path[sequence_cnt].poses.push_back(pose_stamped);
// //                 path[sequence_cnt].header = pose_stamped.header;
//                 
            }
            
        }
        
        mFrameList.push_back(pFrame);
    }
     
     
     
// useless staff---------------------------------------------------------
// void LoopClosing::setBackEnd(shared_ptr<BackendInterface> pBackEnd)
// {
//   if(pBackEnd!=nullptr)
//   {
//     mpBackEnd = pBackEnd;
//   }
//   else
//   {
//     LOG(INFO)<<"pBackEnd null"<<endl;
//   }
//     
// }


  
}
