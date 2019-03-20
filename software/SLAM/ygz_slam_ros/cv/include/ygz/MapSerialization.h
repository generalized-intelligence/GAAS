#ifndef MAPSERIALIZATION_H
#define MAPSERIALIZATION_H

#include "ygz/serialization.h"
#include "ygz/Frame.h"
#include "ygz/Tracker.h"

#include "opencv/cv.h"

#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

namespace ygz
{
    
    class MapSerialization
    {
        
    public:
        typedef  std::tuple<std::vector<cv::KeyPoint>, std::vector<cv::Point3d>, cv::Mat> SceneFrame;
        
        MapSerialization();
        
        void addFrame(shared_ptr<Frame> mpCurrentFrame);
        
        void serialize(string& path);
        
        MapSerialization deserialize(string& path);
        
        // test deserializatin result by displaying the size of the map vector, size of the first scene and the size of the last scene
        void test();
        
        
        /////////////////////////////////// serialization////////////////////////////////////
        BOOST_SERIALIZATION_SPLIT_MEMBER()
        template <class Archive>
        void save (Archive & ar, const unsigned int version) const
        {
            ar & mVecSceneFrames;
            //ar & point_cloud_of_scene;
        }

        template <class Archive>
        void load (Archive & ar, const unsigned int version)
        {
            ar & mVecSceneFrames;

        }
        /////////////////////////////////// serialization////////////////////////////////////
        
        inline int getSize()
        {
            return mVecSceneFrames.size();
        }
        
    public:
        
        size_t mIndex = 0;
        std::vector<std::vector<cv::KeyPoint> > vec_p2d;
        std::vector<std::vector<cv::Point3d> > vec_p3d;
        std::vector <cv::Mat> point_desps;
        cv::Mat m_RT_Scene_Fix = cv::Mat::eye(4,4,CV_32F);

        
    public:
        
        std::vector<SceneFrame> mVecSceneFrames;
        
        cv::Ptr<cv::ORB> mOrb;
        SceneFrame mCurrentSceneFrame;
        cv::Mat mCurrentImage;
        vector<cv::KeyPoint> mCurrentKPs;
        vector<cv::Point3d> mCurrentMPs;
        cv::Mat mCurrentDesp;
        
    };
    
}






#endif
