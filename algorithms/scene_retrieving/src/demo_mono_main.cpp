//receive ros msg:1.image_l,image_r   2.position.
//output:scene struct.
#include <memory>
#include <opencv2/opencv.hpp>
#include "scene_retrieve.h"


using namespace std;
int main(int argc,char** argv)
{
    
    //NOTE simple test, for fast serialization test
    std::shared_ptr<Scene> pSceneTest(new Scene());
    string test_scene_path = "../../scene.scn";
    pSceneTest->loadFile(test_scene_path);
    pSceneTest->test();
    
    
    if (argc!=4)
    {
        cout<<"Usage: demo [scene_file_path] [voc_path] [image_path] [camera_mat_file_path]"<<endl;
    }

    std::string scene_path(argv[1]), voc_file_path(argv[2]), img_path(argv[3]), Q_mat_path(argv[4]);
    cv::FileStorage fsSettings(Q_mat_path, cv::FileStorage::READ);
    
    cout<<"scene path: "<<scene_path<<endl;
    cout<<"voc_file_path: "<<voc_file_path<<endl;
    cout<<"img_path: "<<img_path<<endl;
    cout<<"Q_mat_path: "<<Q_mat_path<<endl;
    
    cv::Mat Q_mat;
    fsSettings["Q_mat"] >> Q_mat;

    std::shared_ptr<Scene> pScene(new Scene());
    pScene->loadFile(scene_path);
    pScene->test();
    
    if(pScene->hasScale == false)
    {
        cout <<"Scene has no scale info.Can not do matching."<<endl;
        return -1;
    }

    cv::Mat RT_mat;
    bool match_success;


    //NOTE define cameraMatrix
    cv::Mat cameraMatrix;

    std::shared_ptr<SceneRetriever> pSceneRetriever(new SceneRetriever());
    pSceneRetriever->retrieveSceneWithScaleFromMonoImage(cv::imread(img_path),cameraMatrix ,RT_mat, match_success);
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








