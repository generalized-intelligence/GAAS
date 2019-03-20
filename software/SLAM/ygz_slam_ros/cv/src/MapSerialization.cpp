#include "ygz/MapSerialization.h"




namespace ygz
{
    
    //typedef  std::tuple<std::vector<cv::KeyPoint>, std::vector<cv::Point3d>, cv::Mat> SceneFrame;
    
    MapSerialization::MapSerialization()
    { 
        cout<<"MapSerialization initialized!"<<endl;
        mOrb = cv::ORB::create();
    };
        
    
    void MapSerialization::addFrame(shared_ptr<Frame> mpCurrentFrame)
    {
        
        vector<cv::KeyPoint> KPs;
        vector<cv::Point3d> MPs;
        
        std::tuple<vector<cv::KeyPoint>, vector<cv::Point3d> > KpMp = mpCurrentFrame->fetchKeyPointAndMapPoint();
        KPs = std::get<0>(KpMp);
        MPs = std::get<1>(KpMp);
        
        mCurrentImage = mpCurrentFrame->mImLeft;
        
        //mOrb->detectAndCompute(mCurrentImage, cv::Mat Mask, ptr_frameinfo->keypoints,ptr_frameinfo->descriptors);
        mOrb->compute(mCurrentImage, KPs, mCurrentDesp);
        mCurrentSceneFrame = std::make_tuple(KPs, MPs, mCurrentDesp);
        mVecSceneFrames.push_back(mCurrentSceneFrame);

    }
    
    
    void MapSerialization::test()
    {
        typedef  std::tuple<std::vector<cv::KeyPoint>, std::vector<cv::Point3d>, cv::Mat> SceneFrame;
        SceneFrame first_scene, last_scene;
        first_scene = mVecSceneFrames.front();
        last_scene = mVecSceneFrames.back();
        
        
        cout<<"---------------------------------------------------------------------------------------------------------------"<<endl;
        cout<<"MapSerialization test, mVecSceneFrames size: "<<mVecSceneFrames.size()<<endl;
        cout<<"MapSerialization test, first scene tuple size: "<<std::get<0>(first_scene).size()<<", "<<std::get<1>(first_scene).size()<<endl;
        cout<<"MapSerialization test, last scene tuple size: "<<std::get<0>(last_scene).size()<<", "<<std::get<1>(last_scene).size()<<endl;
        cout<<"---------------------------------------------------------------------------------------------------------------"<<endl;
        
    }
    
    
    
    void MapSerialization::serialize(string& path)
    {
        
        std::ofstream ofs(path);

        {
            boost::archive::text_oarchive oa(ofs);
            oa << *this;
        }
        
        cout<<"MapSerialization serialization finished, current SceneFrame vector size: "<<this->mVecSceneFrames.size()<<endl;

    }
    
    //Load from binary file
    MapSerialization MapSerialization::deserialize(string& path)
    {

        std::ifstream ifs(path);
        MapSerialization RecovedredScene;

        {
            boost::archive::text_iarchive ia(ifs);
            ia >> RecovedredScene;
            cout << "Deserialization finished" << endl;
            this->test();
        }

        return RecovedredScene;
    }
    
    
}
