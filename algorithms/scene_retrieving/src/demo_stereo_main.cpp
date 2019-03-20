//receive ros msg:1.image_l,image_r   2.position.
//output:scene struct.
#include <memory>
#include <opencv2/opencv.hpp>
#include "scene_retrieve.h"
#include <iostream>


using namespace std;

int main(int argc,char** argv)
{
    
    //NOTE simple test, for fast serialization test
    std::shared_ptr<Scene> pSceneTest(new Scene());
    string test_scene_path = "../../test.scene";
    pSceneTest->loadFile(test_scene_path);
    pSceneTest->test();
    
    
    if (argc!=4)
    {
        cout<<"Usage: demo [scene_file_path] [l_image_path] [r_image_path] [Q_mat_file_path]"<<endl;
    }

    std::string scene_path(argv[1]), l_img_path(argv[2]), r_img_path(argv[3]), Q_mat_path;
    cv::FileStorage fsSettings(Q_mat_path, cv::FileStorage::READ);
    cv::Mat Q_mat;
    fsSettings["Q_mat"] >> Q_mat;

    std::shared_ptr<Scene> pScene(new Scene());
    pScene->loadFile(scene_path);
    pScene->test();

    cv::Mat RT_mat;
    bool match_success;

    std::shared_ptr<SceneRetriever> pSceneRetriever(new SceneRetriever());

    pSceneRetriever->retrieveSceneFromStereoImage(cv::imread(l_img_path), cv::imread(r_img_path), Q_mat, RT_mat, match_success);
    if(match_success)
    {
        cout<<"Match success!\tRT mat:"<<RT_mat<<endl;
    }
    else
    {
        cout<<"Match failed!"<<endl;
    }

    return 0;

}








