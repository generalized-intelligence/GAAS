//receive ros msg:1.image_l,image_r   2.position.
//output:scene struct.
#include <memory>
#include <opencv2/opencv.hpp>
#include "scene_retrieve.h"


using namespace std;
int main(int argc,char** argv)
{
    //ros init
    ros::init(argc, argv, "scene_retrieve");

    if (argc!=4)
    {
        cout<<"Usage: demo [scene_file_path] [voc_file_path] [image_path] [Q_mat_file_path]"<<endl;
    }

    std::string scene_path(argv[1]), voc_file_path(argv[2]) , l_img_path(argv[3]), Q_mat_path(argv[4]);//l_img is the only image we need.

    cout<<"scene path: "<<scene_path<<endl;
    cout<<"voc_file_path: "<<voc_file_path<<endl;
    cout<<"l_img_path: "<<l_img_path<<endl;
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


    int recalled_result=0;

    // test case for mono case
    recalled_result=0;
    for (int i=0; i<image_num; i++)
    {
        cout<<"retrieveSceneFromStereoImage !"<<endl;
        cv::Mat left_image = cv::imread(left_image_path[i]);

        if(left_image.empty() && Q_mat.empty())
        {
            cout<<"left or Q_mat is empty!"<<endl;
            continue;
        }
        else
        {
            int inliers = pSceneRetriever->retrieveSceneWithScaleFromMonoImage(left_image, Q_mat, RT_mat, match_success);

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








