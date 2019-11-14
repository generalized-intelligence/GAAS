//receive ros msg:1.image_l,image_r   2.position.
//output:scene struct.
#include <memory>
#include <opencv2/opencv.hpp>
#include "scene_retrieve.h"
#include <iostream>

#include <ros/ros.h>

using namespace std;

int main(int argc,char** argv)
{

    //ros init
    ros::init(argc, argv, "scene_retrieve");
    
    if (argc!=5)
    {
        cout<<"Usage: demo [scene_file_path] [voc_file_path] [l_image_path] [r_image_path] [Q_mat_file_path]"<<endl;
    }


    std::string scene_path(argv[1]), voc_file_path(argv[2]) , l_img_path(argv[3]), r_img_path(argv[4]), Q_mat_path(argv[5]);
    
    cout<<"scene path: "<<scene_path<<endl;
    cout<<"voc_file_path: "<<voc_file_path<<endl;
    cout<<"l_img_path: "<<l_img_path<<endl;
    cout<<"r_img_path: "<<r_img_path<<endl;
    cout<<"Q_mat_path: "<<Q_mat_path<<endl;
    
    cv::FileStorage fsSettings(Q_mat_path, cv::FileStorage::READ);

    cv::Mat Q_mat;
    fsSettings["Q_mat"] >> Q_mat;

    if (Q_mat.empty())
    {
        cout<<"Q mat empty, exit."<<endl;
        return -1;
    }
    

    cout<<"Q_mat: "<<endl<<Q_mat<<endl;
    
    cv::Mat RT_mat = (cv::Mat_<float >(4,4) << 1, 0, 0, 0,
                                               0, 1, 0, 1,
                                               0, 0, 1, 0,
                                               0, 0, 0, 1);

    bool match_success;

    std::shared_ptr<SceneRetriever> pSceneRetriever(new SceneRetriever(voc_file_path, scene_path));

    int image_num = 5000;
    vector<string> left_image_path, right_image_path;
    for (int i=0; i<image_num; i++)
    {
        string left_path = "./image/left/" + to_string(i) + ".png";
        left_image_path.push_back(left_path);
        
        string right_path = "./image/right/" + to_string(i) + ".png";
        right_image_path.push_back(right_path);
    }


    pSceneRetriever->setImageVecPath(left_image_path, 1);
    pSceneRetriever->setImageVecPath(right_image_path, 0);


    // test case for stereo image
    int recalled_result=0;
    for (int i=0; i<image_num; i++)
    {
        cout<<"retrieveSceneFromStereoImage 1"<<endl;
        cv::Mat left_image = cv::imread(left_image_path[i]);
        cv::Mat right_image = cv::imread(right_image_path[i]);

        if(left_image.empty() && right_image.empty() && Q_mat.empty())
        {
            cout<<"left or right image or Q_mat is empty!"<<endl;
            continue;
        }
        else
        {

            int* loop_index;
            float fitness_score = pSceneRetriever->retrieveSceneFromStereoImage(left_image, right_image, Q_mat, RT_mat, match_success, loop_index);
            cout<<"retrieveSceneFromStereoImage 2"<<endl;

            if(fitness_score < -1.0)
            {
              continue;
            }

            if(match_success)
            {
                cout<<to_string(i)<<" Match success!\tRT mat:"<<RT_mat<<endl;
                recalled_result++;
            }
        }

        cout<<"retrieveSceneFromStereoImage recalled result: "<<recalled_result<<endl;
        cout<<"retrieveSceneFromStereoImage recall: "<<(recalled_result/image_num)<<endl;
    }



    
    return 0;
}








