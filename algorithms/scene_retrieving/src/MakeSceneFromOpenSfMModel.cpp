
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
    map<string,mat> img2Rotation,img2Translation;
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
    
    for(json::iterator it = shots.begin();it!=shots.end();++it)
    {
        pScene->addFrame(....);
    }



/*
OpenSfM opensfm/tracking.py
line 38:

def create_tracks_graph(features, colors, matches, config):
    """Link matches into tracks."""
    logger.debug('Merging features onto tracks')
    uf = UnionFind()
    for im1, im2 in matches:
        for f1, f2 in matches[im1, im2]:
            uf.union((im1, f1), (im2, f2))

    sets = {}
    for i in uf:
        p = uf[i]
        if p in sets:
            sets[p].append(i)
        else:
            sets[p] = [i]

    min_length = config['min_track_length']
    tracks = [t for t in sets.values() if _good_track(t, min_length)]
    logger.debug('Good tracks: {}'.format(len(tracks)))

    tracks_graph = nx.Graph()
    for track_id, track in enumerate(tracks):
        for image, featureid in track:
            if image not in features:
                continue
            x, y = features[image][featureid]
            r, g, b = colors[image][featureid]
            tracks_graph.add_node(str(image), bipartite=0)
            tracks_graph.add_node(str(track_id), bipartite=1)
            tracks_graph.add_edge(str(image),
                                  str(track_id),
                                  feature=(float(x), float(y)),
                                  feature_id=int(featureid),
                                  feature_color=(float(r), float(g), float(b)))

    return tracks_graph

*/
    }

}



int main(int argc,char** argv)
{
    if(argc<2)
    {
        cout<<"Usage: MakeSceneFromOpenSfMModel OpenSfM_project_dir"<<endl;
        return -1;
    }
    string project_path(argv[1]);
    pScene = new Scene();
    return 0;
}
