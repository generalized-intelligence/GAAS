#include "Frame.h"
#include "FeatureFrontEndCV.h"



namespace mcs
{
    using namespace std;
    void doFindEssentialMatrix(vector<KeyPoint>& prev,vector<KeyPoint>& next,cvMat& output_essential_mat);

    void doFindEssentialMatrix(Frame& f_prev,Frame& f_next);
    void doFindFundamentalMatrix();
    void doSolvePnP()
    {
        
    }






}
