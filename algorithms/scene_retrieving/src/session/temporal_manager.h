

class MappingSession();//Avoid scene shifting when do retreving/mapping between multiple scenes.
void InitMappingSession(MultiSceneRetriever &s,MappingSession& m);


class MappingSession // to manage the temporal relation of multi scene.
{
public:
    MappingSession();
        
private:

//session state
    bool session_locked;
//motion dynamics state
    quarternion state_estimated;
    double gps_accurate_longitude;
    double gps_accurate_latitude;
    double height_accurate;
//scene selection state
    shared_ptr<pScene> selected_scene;
    double visual_matching_dynamic_threshold;
};
