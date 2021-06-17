#include "IMUPreintLib.h"
#include <glog/logging.h>

#include "offline_dataset.h"
int main(int argc, char** argv)
{
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("imu_preint_node");
    SequenceDataset sd;
    loadDatasetFromBag(sd);
    run_all_pipeline(argc,argv,sd);
    return 0;
}
