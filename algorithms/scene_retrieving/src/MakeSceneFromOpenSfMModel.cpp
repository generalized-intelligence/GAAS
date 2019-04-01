
#include <iostream>
///#include <io.h>
#include <math.h>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>


#include <math.h>


//add scene retrieving.
#include "scene_retrieve.h"
#include "LoopClosingManager.h"
#include "nlohmann/json.hpp"


using namespace std;
using namespace nlohmann;
//Scene Pointer
//Scene* pScene = NULL;

cv::Ptr<cv::ORB> orb;
void ExtractFeaturesFromCertainKeyPoints(const cv::Mat& img_in,const vector<cv::Point2f> kps_in,vector<cv::Mat>& desp_output)
{
    //reference loop closing manager,extract orb feature of these points.
    cv::Mat gray;
    cv::Mat desp;
    cv::cvtColor(img_in,gray,cv::COLOR_RGB2GRAY);
    vector<cv::KeyPoint> kp_vec;
    cv::KeyPoint::convert(kps_in,kp_vec);
    orb->compute(gray,kp_vec,desp);
    cout<<"kps_in.size():"<<kps_in.size()<<"\tdesp.rows:"<<desp.rows<<endl;
    desp_output.resize(desp.rows);
    for(int r=0;r<desp.rows;r++) desp_output[r]=desp.rowRange(r,r+1).clone();
    return;
}


std::shared_ptr<Scene> MakeSceneFromPath(const string& path)
{
    std::shared_ptr<Scene> pScene(new Scene());
    LoopClosingManager lcm();
    //step<1> parse json.
    string jsonpath(path.c_str());
    jsonpath += "/reconstruction.json";
    cout<<"Using json path: "<<jsonpath<<endl;
    
    std::ifstream ifstr_json( jsonpath);
    
//     std::ifstream ifstr_json( jsonpath.c_str() );
    
    json reconstruction_j;
    
    
    ifstr_json >> reconstruction_j;
    
    
    auto shots = reconstruction_j[0]["shots"];

    auto img2Rotation = new map<string, cv::Mat>;
    auto img2Translation = new map<string, cv::Mat>;
    

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
        (*img2Rotation)[img_filename] = rotation_mat;
        (*img2Translation)[img_filename] = translation_mat;
        //std::cout << it.key() << " : " << it.value() << "\n";
    }
    
    cout<<"??? 6"<<endl;
    
    //step<2> build graph,do triangulate via these features.Load "features/xxxnpz and get keypoints info."
    map<string, vector<cv::Mat> >* pMapdesp = new map<string, vector<cv::Mat> >;
    map<string, vector<cv::Mat> >& img2kpDesps = *pMapdesp;
    //map<string, vector<cv::Mat> > img2kpDesps;// = *pMapdesp;    
    
    //map<string, vector<cv::KeyPoint> >* pMap2d = new map<string, vector<cv::KeyPoint> >;
    //map<string, vector<cv::KeyPoint> > &img2Kp2Ds = *pMap2d;
    map<string, vector<cv::KeyPoint> > img2Kp2Ds;
    
    map<string,vector<cv::Point3d> >* pMap3d = new map<string,vector<cv::Point3d> >; 
    map<string,vector<cv::Point3d>> &img2Kp3ds = *pMap3d;
    for(json::iterator it = shots.begin();it!=shots.end();++it)
    {
        string img_filename = it.key();
        std::ifstream ifstr_img_keypoints_json((path+"/features/"+img_filename+".npz.json").c_str());
        json img_kp_json;
        ifstr_img_keypoints_json>> img_kp_json;
        vector<cv::Point2f> kps;
        vector<cv::Mat> current_img_desps;
        for(json::iterator it2 = img_kp_json.begin();it2!=img_kp_json.end();++it2)
        {
            string point_index = it2.key();//make keypoints.
            string keypoint_x_str,keypoint_y_str;
	    /*for(json::iterator it_inner = img_kp_json[point_index].begin();it_inner!=img_kp_json[point_index].end();++it_inner)
	    {
	      cout<<"Inner object:"<<*it_inner;
	    }*/
            keypoint_x_str = img_kp_json[point_index][0].get<string>();
            keypoint_y_str = img_kp_json[point_index][1].get<string>();
	    float kpx,kpy;
	    kpx = std::stof(keypoint_x_str);
	    kpy = std::stof(keypoint_y_str);
	    //cout<<"kpx,kpy:"<<kpx<<" "<<kpy<<endl;
            kps.push_back(cv::Point2f(kpx,kpy));
        }
        cv::Mat curr_img = cv::imread(path+"/images/"+img_filename);
        ExtractFeaturesFromCertainKeyPoints(curr_img,kps,current_img_desps);
        img2kpDesps[img_filename] = current_img_desps;
	/*
	std::vector<cv::KeyPoint> *kps_ = new vector<cv::KeyPoint>;
	cv::KeyPoint::convert(kps,*kps_);
        img2Kp2Ds[img_filename] = *kps_;*/
	std::vector<cv::KeyPoint> kps_;
	cv::KeyPoint::convert(kps,kps_);
        img2Kp2Ds[img_filename] = kps_;
	
        img2Kp3ds[img_filename] = vector<cv::Point3d>();
        img2Kp3ds[img_filename].resize(img2Kp2Ds[img_filename].size());
    }
    //step<3>.Get 3d position of these points,and make a frame of scene.
    //open undistorted_tracks.csv. actually it's a tsv file.
    ifstream track_ifstr((path+"/undistorted_tracks.csv").c_str());
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
    std::map<string,std::vector<int> > map_index_with_3d_pos;
    const auto &points_obj = reconstruction_j[0]["points"];
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
	//string px,py,pz;
	cout<<endl;
	cout<<"curr_line[0]:"<<curr_line[0]<<endl;
        cout<<"at line_index:"<<line_index<<endl;
	cout<<"track_id:"<<track_id_str<<endl;
	cout<<"feature_id:"<<feature_id_str<<endl;
	//auto z = reconstruction_j[0];
	//auto a = reconstruction_j[0]["points"];
	cout <<"bb"<<endl;
	//auto b = reconstruction_j[0]["points"][track_id_str];
	cout <<"cc"<<endl;
	//auto c = reconstruction_j[0]["points"][track_id_str]["coordinates"];
        cout <<"0"<<endl;

	if(points_obj.find(track_id_str)==points_obj.end() )
	{
	  //is a outlier.
	  cout <<"is a outlier"<<endl;
	  continue;
	}
	else
	{
	  cout<<"is a inlier!"<<endl;
	}
	double px = reconstruction_j[0]["points"][track_id_str]["coordinates"][0].get<double>();
	cout<<"1"<<endl;
	double py = reconstruction_j[0]["points"][track_id_str]["coordinates"][1].get<double>();
	cout<<"2"<<endl;
	double pz = reconstruction_j[0]["points"][track_id_str]["coordinates"][2].get<double>();
	cout<<"3"<<endl;
	cout <<"img2Kp3ds[img_filename].size:"<<img2Kp3ds[img_filename].size()<<"\timg2Kp2Ds[img_filename].size():"<<img2Kp2Ds[img_filename].size()<<endl;
	if(img2Kp2Ds[img_filename].size() == 0)
	{
	  cout <<"this image is not inside of reconstruction[0],ignored!"<<endl;
	  continue;
	}
        img2Kp3ds[img_filename][feature_id] = cv::Point3d(px,py,pz);
	
	cout<<"point added!"<<endl;
	//map_index_with_3d_pos[img_filename].push_back(feature_id);//Unused now.
    }
    cout<<"3d points loaded."<<endl;
    for (json::iterator it = shots.begin(); it != shots.end(); ++it)
    {
        //cv::Mat rotation_mat,translation_mat;
        string img_filename = it.key();
        cv::Mat temp_desp_mat(img2kpDesps[img_filename].size(),//rows
                    32,//cols
                        0
        );
        cout<<"temp_desp_mat:"<<temp_desp_mat.cols<<"|"<<temp_desp_mat.rows<<endl;
        cout<<"Image:"<<img_filename<<img2kpDesps[img_filename].size()<<";"<<img2kpDesps[img_filename][0].rows<<","<<img2kpDesps[img_filename][0].cols<<endl;
        
        for(int index = 0;index< img2kpDesps[img_filename].size();index++)
        {
            cout<<"index:"<<index<<endl;
            cv::Mat& temp = img2kpDesps[img_filename][index];
            cout<<temp.cols<<"|"<<temp.rows<<endl;
            //temp_desp_mat.rowRange(index,index+1) = img2kpDesps[img_filename][index].row(0);//img2kpDesps[img_filename][index].rowRange(0,1);//.clone();
            temp_desp_mat.row(index) = img2kpDesps[img_filename][index].row(0);//img2kpDesps[img_filename][index].rowRange(0,1);//.clone();
        }
    
        //NOTE method 1, this will serialize kps, mps and desp;
        //pScene->addFrame(img2Kp2Ds[img_filename], img2Kp3ds[img_filename], temp_desp_mat.clone());
        
        //NOTE method 2, this will serialize kps, mps, desps, R and t
        cv::Mat R = (cv::Mat_<double>(3,3) << 0, -1.1, 0, -1.2, 5.3, -1.4, 0, -1.5, 0);
        cv::Mat T = (cv::Mat_<double>(1,3) << 0, -1.5, 0);
        cout<<"Current R and T are: "<<endl<<R<<endl<<T<<endl;
        
        pScene->addFrame(img2Kp2Ds[img_filename], img2Kp3ds[img_filename], temp_desp_mat.clone(), R, T);
        
        
    }
    //step<4>.See if this scene has scale factor.
    
    //pScene->hasScale = false;// for default.
    return pScene;
}



int main(int argc,char** argv)
{
    cout<<"CAUTION:Do not forget set opensfm config feature_type to 'ORB'."<<endl;
    if(argc<3)
    {
        cout<<"Usage: MakeSceneFromOpenSfMModel OpenSfM_project_dir voc_path"<<endl;
        return -1;
    }
    string project_path(argv[1]);
    string voc_path(argv[2]);
    orb = cv::ORB::create();
    std::shared_ptr<Scene>* pp;
    pp = new std::shared_ptr<Scene>;
    *pp= MakeSceneFromPath(project_path);
    (*pp)->saveFile("scene.scene");
    return 0;
}
