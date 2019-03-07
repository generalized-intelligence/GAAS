
#include <iostream>
#include <io.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <math.h>


//add scene retrieving.
#include "scene_retrieve.h"
#include "LoopClosingManager.h"
#include "nlohmann/json.hpp"


using namespace std;
//Scene Pointer
Scene* pScene = NULL;

cv::Ptr<cv::ORB> orb;
void ExtractFeaturesFromCertainKeyPoints(const cv::Mat& img_in,const vector<Point2f> kps_in,vector<cv::Mat>& desp_output)
{
    //reference loop closing manager,extract orb feature of these points.
    vector<cv::KeyPoint> kp_vec;
    cv::KeyPoint::convert(kps_in,kp_vec);
    orb->compute(img_in,kp_vec,desp_output);
    return;
}


void MakeSceneFromPath(const string& path)
{
    LoopClosingManager lcm();
    //step<1> parse json.
    string jsonpath(path.c_str());
    jsonpath+="reconstruction.json";
    std::ifstream ifstr_json( jsonpath.c_str() );
    json reconstruction_j;
    ifstr_json >> reconstruction_j;
    auto shots = json["shots"];
    map<string,Mat> img2Rotation,img2Translation;
    for (json::iterator it = shots.begin(); it != shots.end(); ++it) 
    {
        cv::Mat rotation_mat,translation_mat;
        string img_filename = it.key();
        cv::Mat rvec(1,3,CV_32F);
        rvec.at<float>(0,0) = (float)shots[img_filename]["rotation"][0].get<double>();
        rvec.at<float>(0,1) = (float)shots[img_filename]["rotation"][1].get<double>();
        rvec.at<float>(0,2) = (float)shots[img_filename]["rotation"][2].get<double>();
        cv::Rodrigues(rvec,rotation_mat); // get 3x3 rotation mat.

        //cv::Mat cur_image;
        //cv::imread(path+"images"+img_filename,cur_image);
        //ptr_frameinfo p = lcm.extractFeature(cur_image);
        img2Rotation[img_filename] = rotation_mat;
        img2Translation[img_filename] = translation_mat;
        //std::cout << it.key() << " : " << it.value() << "\n";
    }
    //step<2> build graph,do triangulate via these features.Load "features/xxxnpz and get keypoints info."
    map<string, vector<cv::Mat> > img2kpDesps;
    map<string, vector<cv::KeyPoint> > img2Kp2Ds;
    map<string,vector<cv::Point3d>> img2Kp3ds;
    for(json::iterator it = shots.begin();it!=shots.end();++it)
    {
        string img_filename = it.key();
        std::ifstream ifstr_img_keypoints_json(img_filename.c_str());
        json img_kp_json;
        ifstr_img_keypoints_json>> img_kp_json;
        vector<cv::Point2f> kps;
        vector<cv::Mat> current_img_desps;
        for(json::iterator it = img_kp_json.begin();it!=img_kp_json.end();++it)
        {
            string point_index = it.key();//make keypoints.
            float keypoint_x,keypoint_y;
            keypoint_x = img_kp_json[point_index][0].get<float>();
            keypoint_y = img_kp_json[point_index][1].get<float>();
            kps.push_back(Point2f(keypoint_x,keypoint_y));
        }
        Mat curr_img;
        imread(img_filename,curr_img);
        ExtractFeaturesFromCertainKeyPoints(curr_img,kps,current_img_desps);
        img2kpDesps[img_filename] = current_img_desps;
        img2Kp2Ds[img_filename] = kps;
        img2Kp3ds[img_filename] = vector<cv::Point3d>;
        img2Kp3ds[img_filename].resize(img2Kp2Ds.size());
    }
    //step<3>.Get 3d position of these points,and make a frame of scene.
    //open undistorted_tracks.csv. actually it's a tsv file.
    ifstream track_ifstr("undistorted_track.csv");
    vector<vector <string> >fields;
    string line;
    while(getline(track_ifstr,line))
    {
        stringstream sep(line);
        string field;
        fields.push_back(vector<string>());
        while(getline(sep,field,'\t'))
        {
            fields.back().push_back(field);
        }
    }
    for(int line_index=0;line_index<fields.size();line_index++) // traverse all tracks to find point 3d coordinate.
    {
        vector<string>& curr_line = fields[line_index];
        string img_filename,track_id_str,feature_id_str;
        int track_id,feature_id;
        img_filename = curr_line[0];
        track_id_str = curr_line[1];
        feature_id_str = curr_line[2];
        track_id = std::stoi(track_id_str);
        feature_id = std::stoi(feature_id_str);
        img2Kp3ds[img_filename][feature_id] = Point3d(
			reconstruction["points"][track_id_str]["coordinates"][0].get<double>(),
			reconstruction["points"][track_id_str]["coordinates"][1].get<double>(),
			reconstruction["points"][track_id_str]["coordinates"][2].get<double>()
						);
    }
    for (json::iterator it = shots.begin(); it != shots.end(); ++it)
    {
        cv::Mat rotation_mat,translation_mat;
        string img_filename = it.key();
        pScene->addFrame(img2Kp2Ds[img_filename],img2Kp3ds[img_filename],img2kpDesps[img_filename]);
    }
    //step<4>.See if this scene has scale factor.
    
    

}



int main(int argc,char** argv)
{
    cout<<"CAUTION:Do not forget set opensfm config feature_type to 'ORB'."<<endl;
    if(argc<2)
    {
        cout<<"Usage: MakeSceneFromOpenSfMModel OpenSfM_project_dir"<<endl;
        return -1;
    }
    string project_path(argv[1]);
    pScene = new Scene();
    orb = cv::ORB::create();
    return 0;
}
