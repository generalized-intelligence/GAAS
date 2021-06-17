/**
 * File: Demo.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DBoW2
 * License: see the LICENSE.txt file
 */

#include <iostream>
#include <vector>

// DBoW2/3
#include "DBoW3.h" // defines OrbVocabulary and OrbDatabase

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/calib3d.hpp"
//#include "opencv2/xfeatures2d.hpp"

using namespace DBoW3;
using namespace std;
using namespace cv;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeatures(vector<vector<cv::Mat > > &features ,vector<vector<cv::KeyPoint> > &kp_list,vector<cv::Mat> &kdesp_list,std::string detect_method = "orb",std::string compute_method = "brief");
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out);
void testVocCreation(const vector<vector<cv::Mat > > &features);
void testDatabase(const vector<vector<cv::Mat > > &features,const std::string db_path,vector<vector<cv::KeyPoint> > &kp_list,vector<cv::Mat> &kdesp_list);


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// number of training images
//const int NIMAGES = 4;
//const int NIMAGES = 1700;

//const int NIMAGES = 1798;
//const int NIMAGES=2250;
const int NIMAGES=1470;


//const int NIMAGES = 40;

const int RET_QUERY_LEN = 4;
//const int TOO_CLOSE_THRES = 15;
const int TOO_CLOSE_THRES = 400;


//const float DB_QUERY_SCORE_THRES = 0.4;//0.5;//0.65;
const float DB_QUERY_SCORE_THRES = 0.0075;//0.015;//0.5;//0.65;
const int STEP1_KP_NUM = 8;//12;
const int STEP2_KP_NUM = 5;//8;

const double ORB_TH_HIGH = 20;//pretty good.//10; pretty good//5;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------
int main();

// ----------------------------------------------------------------------------

void loadFeatures(vector<vector<cv::Mat > > &features ,vector<vector<cv::KeyPoint> > &kp_list,vector<cv::Mat> &kdesp_list,std::string detect_method,std::string compute_method)
{
  features.clear();
  features.reserve(NIMAGES);

  cv::Ptr<cv::ORB> orb = cv::ORB::create();

  //cv::BriefDescriptorExtractor brief; 
  cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
  cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create();
  //cv::Ptr<cv::KAZE> kaze = cv::KAZE::create();

  cout << "Extracting ORB features..." << endl;
  for(int i = 0; i < NIMAGES; ++i)
  {
    stringstream ss;
    ss << "images/image" << i << ".png";

    cv::Mat image = cv::imread(ss.str(), 0);
    cv::Mat mask;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    orb->detectAndCompute(image, mask, keypoints, descriptors);
    //orb->detect(image,keypoints);
    //brief->compute(image,keypoints,descriptors);
    //surf->detectAndCompute(image,mask,keypoints,descriptors);
    //surf->compute(image,keypoints,descriptors);
    //kaze->detectAndCompute(image,mask,keypoints,descriptors);

    features.push_back(vector<cv::Mat >());
    changeStructure(descriptors, features.back());

    
    kp_list.push_back(keypoints);
    kdesp_list.push_back(descriptors);
    if(i%10 == 0)
    {
        cout<<"Processing image " <<i <<"."<<endl;
    }
  }
}

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out)
{
  out.resize(plain.rows);

  for(int i = 0; i < plain.rows; ++i)
  {
    out[i] = plain.row(i);
  }
}

// ----------------------------------------------------------------------------

void testVocCreation(const vector<vector<cv::Mat > > &features)
{
  // branching factor and depth levels 
  const int k = 10;
  const int L = 3;
  const WeightingType weight = TF_IDF;
  const ScoringType score = L1_NORM;

  //OrbVocabulary voc(k, L, weight, score);
  Vocabulary voc(k, L, weight, score);
  //BriefVocabulary voc(k,L,weight,score);

  cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
  voc.create(features);
  cout << "... done!" << endl;

  cout << "Vocabulary information: " << endl
  << voc << endl << endl;

  // lets do something with this vocabulary
  cout << "Matching images against themselves (0 low, 1 high): " << endl;
  BowVector v1, v2;
  for(int i = 0; i < NIMAGES; i++)
  {
    voc.transform(features[i], v1);
    for(int j = i-4; j < i+4; j++)
    {
      if(j<0)
      {
        continue;
      }
      if(j>=NIMAGES)
      {
        continue;

      }
      voc.transform(features[i], v2);
      
      double score = voc.score(v1, v2);
      cout << "Image " << i << " vs Image " << j << ": " << score << endl;
    }
  }

  // save the vocabulary to disk
  cout << endl << "Saving vocabulary..." << endl;
  voc.save("small_voc.yml.gz");
  cout << "Done" << endl;
}



void saveImagePair(int id1,int id2,int &save_index,std::vector<DMatch>& good_matches,vector<vector<cv::KeyPoint> > &kp_list,vector<cv::Mat> &kdesp_list,double score);

// ----------------------------------------------------------------------------

bool match_2_images_flann(int index1,int index2,vector<vector<cv::KeyPoint> > &kp_list,vector<cv::Mat> &kdesp_list,int &save_index,double score)
{
  //TODO:refer VINS KeyFrame::findConnection().



  //using namespace cv::xfeatures2d;
  FlannBasedMatcher matcher = FlannBasedMatcher(makePtr<flann::LshIndexParams>(12,20,2));
  //FlannBasedMatcher matcher = FlannBasedMatcher();
  std::vector< DMatch > matches;
  matcher.match( kdesp_list[index1], kdesp_list[index2], matches );

  cout<<"Matched!"<<endl;


  double max_dist = 0; double min_dist = 100;
  for( int i = 0; i < matches.size(); i++ )
  {
    double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
  cout<<"min_dist:"<<min_dist<<endl;
  std::vector< DMatch > good_matches;

  for( int i = 0; i < matches.size(); i++ )
  {
    if( matches[i].distance <= 2*min_dist && matches[i].distance< ORB_TH_HIGH) // 3.0 too large;2.0 too large.
    {
      good_matches.push_back( matches[i]); 
    }
  }

  std::vector<cv::Point2f> match_points1;
  std::vector<cv::Point2f> match_points2;

  cout<<"Good matches count:"<<good_matches.size()<<"."<<endl;
  if(good_matches.size()<STEP1_KP_NUM) // 8 -> 12
  {
    cout<<"Good matches count:"<<good_matches.size()<<"< 8,MATCH FAILED."<<endl;
    return false;
  }
  for( size_t i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    //cout<<"Step1_"<<i<<endl;
    //cout<<"kp_list size:"<<kp_list.size()<<endl;
    //cout<<"kplist index:"<<index1<<endl;
    //auto a = kp_list[index1];
    //cout<<"2";
    //auto b = good_matches[i];
    //cout <<"3";
    //auto c = good_matches[i].queryIdx;
    //cout<<4;
    match_points1.push_back( kp_list[index1][ good_matches[i].queryIdx ].pt );
    //cout<<"Step2_"<<i<<endl;

    //auto d = kp_list[index2];
    //cout<<5;
    //auto e = good_matches[i].trainIdx;
    //cout<<6;
    match_points2.push_back( kp_list[index2][ good_matches[i].trainIdx ].pt );
  }
  Mat isOutlierMask;
  Mat fundamental_matrix = findFundamentalMat(match_points1, match_points2, FM_RANSAC, 3, 0.99,isOutlierMask);
  int final_good_matches_count = 0;
  std::vector<DMatch> final_good_matches;
  for(int i = 0;i<good_matches.size();i++)
  {
    if (isOutlierMask.at<int>(i)!=0)
    {
      final_good_matches.push_back(good_matches[i]);
      final_good_matches_count++;
    }
    
  }
  if(final_good_matches_count>STEP2_KP_NUM)
  {
    saveImagePair(index1,index2,save_index,final_good_matches,kp_list,kdesp_list,score);
    cout<<"MATCH SUCCESS."<<endl;
    return true;
  }
  return false;

}

void saveImagePair(int id1,int id2,int &save_index,std::vector<DMatch>& good_matches,vector<vector<cv::KeyPoint> > &kp_list,vector<cv::Mat> &kdesp_list,double score)
{
    stringstream ss;
    ss << "images/image" << id1 << ".png"; 
    cv::Mat image = cv::imread(ss.str(), 0);
    
    stringstream ss2;
    ss2<< "images/image"<<id2<<".png";
    cv::Mat image2 = cv::imread(ss2.str(),0);

//
    cv::Mat merged_img;
//    cv::hconcat(image,image2,merged_img);

//use cv::drawMatches replace hconcat.
    
    cv::drawMatches(image,kp_list[id1],image2,kp_list[id2],good_matches,merged_img);
    stringstream output_ss;

    output_ss<<"loops/image"<<save_index<<"__"<<id1<<"_"<<id2<<"___"<<score<<".png";
    cv::imwrite(output_ss.str(),merged_img);
    save_index++;
}


void testDatabase(const vector<vector<cv::Mat > > &features,const std::string db_path,vector<vector<cv::KeyPoint> > &kp_list,vector<cv::Mat> &kdesp_list)
{
  cout << "Creating a small database..." << endl;

  // load the vocabulary from disk
  //OrbVocabulary voc(db_path.c_str());
  //Vocabulary voc(db_path.c_str());
  Vocabulary voc("../orbvoc.dbow3");
  //BriefVocabulary voc(db_path.c_str());
  
  
  //OrbDatabase db(voc, false, 0); // false = do not use direct index
  Database db(voc, false, 0); // false = do not use direct index
  //BriefDatabase db(voc, false, 0); // false = do not use direct index

  // (so ignore the last param)
  // The direct index is useful if we want to retrieve the features that 
  // belong to some vocabulary node.
  // db creates a copy of the vocabulary, we may get rid of "voc" now

  // add images to the database


  int loop_id = 0;
  for(int i = 0; i < NIMAGES; i++)
  {
  

  //cout << "... done!" << endl;

  //cout << "Database information: " << endl << db << endl;

    // and query the database
    cout << "Querying the database: " << endl;

    QueryResults ret;
    db.query(features[i], ret, RET_QUERY_LEN,i-TOO_CLOSE_THRES);

    db.add(features[0]);
    // ret[0] is always the same image in this case, because we added it to the 
    // database. ret[1] is the second best match.

    cout << "Searching for Image " << i << ". " << ret << endl;
    //ret[0].Score;ret[0].Id
    db.add(features[i]);
    
    for(int wind_index =0;wind_index<ret.size();wind_index++)
    {
        if (ret[wind_index].Score>DB_QUERY_SCORE_THRES)
        {
          if (ret[wind_index].Id<i-TOO_CLOSE_THRES)
          {
            if (match_2_images_flann(i,ret[wind_index].Id,kp_list,kdesp_list,loop_id,ret[wind_index].Score)) // check if this loop candidate satisfies Epipolar Geometry constrain.
            {
              cout<<"Loop between ["<<i<<"\t"<<ret[wind_index].Id<<"]"<<endl;
              break; // match one frame only once.
            }
          }
        }
        else
        {
          break;
        }
    }
  }

  cout << endl;

  // we can save the database. The created file includes the vocabulary
  // and the entries added
  cout << "Saving database..." << endl;
  db.save("small_db.yml.gz");
  cout << "... done!" << endl;
  
  // once saved, we can load it again  
  cout << "Retrieving database once again..." << endl;
  //OrbDatabase db2("small_db.yml.gz");
  Database db2("small_db.yml.gz");
  //BriefDatabase db2("small_db.yml.gz");
  
  cout << "... done!" << endl ;//<< db2 << endl;
}

// ----------------------------------------------------------------------------

class LoopClosingManager
{
public:
    LoopClosingManager();
    void addKeyFrame();
    QueryResults queryKeyFrame();
    int saveDB();
    void loadFromDB();
};

LoopClosingManager::LoopClosingManager()
{
}
void addKeyFrame()
{
}
int saveDB()
{
}
void loadFromDB()
{
}

int main()
{
  vector<vector<cv::Mat > > features;

  vector<vector<cv::KeyPoint> > kp_list;
  vector<cv::Mat> kdesp_list;
  loadFeatures(features,kp_list,kdesp_list);  //image to features.

  //testVocCreation(features); //Do not create any voc.Use a pretrained one.

  //wait();

  testDatabase(features,"small_voc.yml.gz",kp_list,kdesp_list);//do test.
  return 0;
}


