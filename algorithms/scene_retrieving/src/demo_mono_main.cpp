//receive ros msg:1.image_l,image_r   2.position.
//output:scene struct.
#include <memory>
#include <opencv2/opencv.hpp>
#include "scene_retrieve.h"


using namespace std;
int main(int argc,char** argv)
{
    if (argc<3)
    {
        cout<<"Usage: demo [scene_file_path] [image_path][camera_mat_file_path]"
    }
    std::string scene_path(argv[1]),img_path(argv[2]),Q_mat_path(argv[3]);
    cv::FileStorage fsSettings(Q_mat_path, cv::FileStorage::READ);
    cv::Mat Q_mat;
    Q_mat <<fsSettings["Q_mat"];
    std::shared_ptr<Scene> pScene(new Scene());
    pScene->loadFile(scene_path);
    if(pScene->hasScale == false)
    {
        cout <<"Scene has no scale info.Can not do matching."<<endl;
        return -1;
    }
    cv::Mat RT_mat;
    bool match_success;
    pScene->retrieveSceneWithScaleFromMonoImage(cv::imread(img_path),cameraMatrix,RT_mat, match_success);
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








