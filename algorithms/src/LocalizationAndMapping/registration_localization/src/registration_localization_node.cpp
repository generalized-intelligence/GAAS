#include "registraion_map_manager.h"
#include "ndt_localization_algorithm.h"
#include "icp_localization_algorithm.h"
#include "GPS_AHRS_sync.h"

class RegistrationLocalizationNode
{
private:
    std::shared_ptr<LocalizationAlgorithmAbstract> pLocalizationAlgorithm;
    RegistrationMapManager::Ptr pMapManager;
    GPS_AHRS_Synchronizer gps_ahrs_sync;
    bool initMapManager(ros::NodeHandle& nh);
    void startLoop();
public:
    void initRegistrationNode()
    {
        shared_ptr<ros::NodeHandle> pNH(new ros::NodeHandle);
        initMapManager(*pNH);
        if(use_icp)
        {
            std::shared_ptr<ICPLocalizationAlgorithm> pICP(new ICPLocalizationAlgorithm);
            pLocalizationAlgorithm = std::static_pointer_cast<LocalizationAlgorithmAbstract>(pICP);
        }
        pLocalizationAlgorithm->initLocalizationModule(*pNH,pMapManager);




        startLoop();
    }

    // Services callback
    bool loadMapServiceCallback();//加载地图 内部函数
    bool unloadMapServiceCallback();//卸载地图 内部函数
    bool reloadNewMapServiceCallback(); //换装新地图 内部函数
    bool getCurrentMapServiceCallback(); //查看当前地图名

    // Subscribers callback
    void lidar_callback()
    {
        if(!pMapManger->mapLoaded())
        {
            LOG(ERROR)<<"No valid map loaded!"<<endl;
            return;
        }
        auto currentMap = pMapManger->getCurrentMap();
        bool localization_result_valid = pLocalizationAlgorithm->doMatchingWithInitialPoseGuess(currentMap,cloud);
        if(localization_result_valid)
        {
            pMapManager->updateMapByLocation(.....)
        }
    }
    void gps_ahrs_callback();
};

int main(int argc,char** argv)
{
    RegistrationLocalizationNode node;
    node.initRegistrationNode();
    return 0;
}
