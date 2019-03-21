#include "multi_scene_retriever.h"
MultiSceneRetriever::MultiSceneRetriever()
{
    this->gps_KDTree = new pcl::PointCloud<pcl::PointXYZ>();
    scene_index = 0;
}
void MultiSceneRetriever::insertSceneIntoKDTree(double longi,double lati,shared_ptr<Scene> pScene)
{
    shared_ptr<MultiSceneNode> pNew(new MultiSceneNode());
    pNew->longitude = longi;
    pNew->lati = lati;
    pNew->pScene = pScene;
    this->insertSceneIntoKDTree(pNew);
}
void MultiSceneRetriever::insertSceneIntoKDTree(shared_ptr<MultiSceneNode> nodeptr)
{
    this->idToNodeMap[this->scene_index] = nodeptr;
    pcl::PointXYZ p;
    p.x = nodeptr->longitude;
    p.y = nodeptr->latitude;
    p.z = 0;
    this->gps_KDTree.points.push_back(p);
    this->scene_index++; // so the index shall be synchronized.
}
