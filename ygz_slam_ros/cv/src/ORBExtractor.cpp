#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fast/fast.h>

#include "ygz/Frame.h"
#include "ygz/Feature.h"
#include "ygz/ORBExtractor.h"

using namespace cv;
using namespace std;

namespace ygz {

    int bit_pattern_31_[256 * 4] =
            {
                    8, -3, 9, 5/*mean (0), correlation (0)*/,
                    4, 2, 7, -12/*mean (1.12461e-05), correlation (0.0437584)*/,
                    -11, 9, -8, 2/*mean (3.37382e-05), correlation (0.0617409)*/,
                    7, -12, 12, -13/*mean (5.62303e-05), correlation (0.0636977)*/,
                    2, -13, 2, 12/*mean (0.000134953), correlation (0.085099)*/,
                    1, -7, 1, 6/*mean (0.000528565), correlation (0.0857175)*/,
                    -2, -10, -2, -4/*mean (0.0188821), correlation (0.0985774)*/,
                    -13, -13, -11, -8/*mean (0.0363135), correlation (0.0899616)*/,
                    -13, -3, -12, -9/*mean (0.121806), correlation (0.099849)*/,
                    10, 4, 11, 9/*mean (0.122065), correlation (0.093285)*/,
                    -13, -8, -8, -9/*mean (0.162787), correlation (0.0942748)*/,
                    -11, 7, -9, 12/*mean (0.21561), correlation (0.0974438)*/,
                    7, 7, 12, 6/*mean (0.160583), correlation (0.130064)*/,
                    -4, -5, -3, 0/*mean (0.228171), correlation (0.132998)*/,
                    -13, 2, -12, -3/*mean (0.00997526), correlation (0.145926)*/,
                    -9, 0, -7, 5/*mean (0.198234), correlation (0.143636)*/,
                    12, -6, 12, -1/*mean (0.0676226), correlation (0.16689)*/,
                    -3, 6, -2, 12/*mean (0.166847), correlation (0.171682)*/,
                    -6, -13, -4, -8/*mean (0.101215), correlation (0.179716)*/,
                    11, -13, 12, -8/*mean (0.200641), correlation (0.192279)*/,
                    4, 7, 5, 1/*mean (0.205106), correlation (0.186848)*/,
                    5, -3, 10, -3/*mean (0.234908), correlation (0.192319)*/,
                    3, -7, 6, 12/*mean (0.0709964), correlation (0.210872)*/,
                    -8, -7, -6, -2/*mean (0.0939834), correlation (0.212589)*/,
                    -2, 11, -1, -10/*mean (0.127778), correlation (0.20866)*/,
                    -13, 12, -8, 10/*mean (0.14783), correlation (0.206356)*/,
                    -7, 3, -5, -3/*mean (0.182141), correlation (0.198942)*/,
                    -4, 2, -3, 7/*mean (0.188237), correlation (0.21384)*/,
                    -10, -12, -6, 11/*mean (0.14865), correlation (0.23571)*/,
                    5, -12, 6, -7/*mean (0.222312), correlation (0.23324)*/,
                    5, -6, 7, -1/*mean (0.229082), correlation (0.23389)*/,
                    1, 0, 4, -5/*mean (0.241577), correlation (0.215286)*/,
                    9, 11, 11, -13/*mean (0.00338507), correlation (0.251373)*/,
                    4, 7, 4, 12/*mean (0.131005), correlation (0.257622)*/,
                    2, -1, 4, 4/*mean (0.152755), correlation (0.255205)*/,
                    -4, -12, -2, 7/*mean (0.182771), correlation (0.244867)*/,
                    -8, -5, -7, -10/*mean (0.186898), correlation (0.23901)*/,
                    4, 11, 9, 12/*mean (0.226226), correlation (0.258255)*/,
                    0, -8, 1, -13/*mean (0.0897886), correlation (0.274827)*/,
                    -13, -2, -8, 2/*mean (0.148774), correlation (0.28065)*/,
                    -3, -2, -2, 3/*mean (0.153048), correlation (0.283063)*/,
                    -6, 9, -4, -9/*mean (0.169523), correlation (0.278248)*/,
                    8, 12, 10, 7/*mean (0.225337), correlation (0.282851)*/,
                    0, 9, 1, 3/*mean (0.226687), correlation (0.278734)*/,
                    7, -5, 11, -10/*mean (0.00693882), correlation (0.305161)*/,
                    -13, -6, -11, 0/*mean (0.0227283), correlation (0.300181)*/,
                    10, 7, 12, 1/*mean (0.125517), correlation (0.31089)*/,
                    -6, -3, -6, 12/*mean (0.131748), correlation (0.312779)*/,
                    10, -9, 12, -4/*mean (0.144827), correlation (0.292797)*/,
                    -13, 8, -8, -12/*mean (0.149202), correlation (0.308918)*/,
                    -13, 0, -8, -4/*mean (0.160909), correlation (0.310013)*/,
                    3, 3, 7, 8/*mean (0.177755), correlation (0.309394)*/,
                    5, 7, 10, -7/*mean (0.212337), correlation (0.310315)*/,
                    -1, 7, 1, -12/*mean (0.214429), correlation (0.311933)*/,
                    3, -10, 5, 6/*mean (0.235807), correlation (0.313104)*/,
                    2, -4, 3, -10/*mean (0.00494827), correlation (0.344948)*/,
                    -13, 0, -13, 5/*mean (0.0549145), correlation (0.344675)*/,
                    -13, -7, -12, 12/*mean (0.103385), correlation (0.342715)*/,
                    -13, 3, -11, 8/*mean (0.134222), correlation (0.322922)*/,
                    -7, 12, -4, 7/*mean (0.153284), correlation (0.337061)*/,
                    6, -10, 12, 8/*mean (0.154881), correlation (0.329257)*/,
                    -9, -1, -7, -6/*mean (0.200967), correlation (0.33312)*/,
                    -2, -5, 0, 12/*mean (0.201518), correlation (0.340635)*/,
                    -12, 5, -7, 5/*mean (0.207805), correlation (0.335631)*/,
                    3, -10, 8, -13/*mean (0.224438), correlation (0.34504)*/,
                    -7, -7, -4, 5/*mean (0.239361), correlation (0.338053)*/,
                    -3, -2, -1, -7/*mean (0.240744), correlation (0.344322)*/,
                    2, 9, 5, -11/*mean (0.242949), correlation (0.34145)*/,
                    -11, -13, -5, -13/*mean (0.244028), correlation (0.336861)*/,
                    -1, 6, 0, -1/*mean (0.247571), correlation (0.343684)*/,
                    5, -3, 5, 2/*mean (0.000697256), correlation (0.357265)*/,
                    -4, -13, -4, 12/*mean (0.00213675), correlation (0.373827)*/,
                    -9, -6, -9, 6/*mean (0.0126856), correlation (0.373938)*/,
                    -12, -10, -8, -4/*mean (0.0152497), correlation (0.364237)*/,
                    10, 2, 12, -3/*mean (0.0299933), correlation (0.345292)*/,
                    7, 12, 12, 12/*mean (0.0307242), correlation (0.366299)*/,
                    -7, -13, -6, 5/*mean (0.0534975), correlation (0.368357)*/,
                    -4, 9, -3, 4/*mean (0.099865), correlation (0.372276)*/,
                    7, -1, 12, 2/*mean (0.117083), correlation (0.364529)*/,
                    -7, 6, -5, 1/*mean (0.126125), correlation (0.369606)*/,
                    -13, 11, -12, 5/*mean (0.130364), correlation (0.358502)*/,
                    -3, 7, -2, -6/*mean (0.131691), correlation (0.375531)*/,
                    7, -8, 12, -7/*mean (0.160166), correlation (0.379508)*/,
                    -13, -7, -11, -12/*mean (0.167848), correlation (0.353343)*/,
                    1, -3, 12, 12/*mean (0.183378), correlation (0.371916)*/,
                    2, -6, 3, 0/*mean (0.228711), correlation (0.371761)*/,
                    -4, 3, -2, -13/*mean (0.247211), correlation (0.364063)*/,
                    -1, -13, 1, 9/*mean (0.249325), correlation (0.378139)*/,
                    7, 1, 8, -6/*mean (0.000652272), correlation (0.411682)*/,
                    1, -1, 3, 12/*mean (0.00248538), correlation (0.392988)*/,
                    9, 1, 12, 6/*mean (0.0206815), correlation (0.386106)*/,
                    -1, -9, -1, 3/*mean (0.0364485), correlation (0.410752)*/,
                    -13, -13, -10, 5/*mean (0.0376068), correlation (0.398374)*/,
                    7, 7, 10, 12/*mean (0.0424202), correlation (0.405663)*/,
                    12, -5, 12, 9/*mean (0.0942645), correlation (0.410422)*/,
                    6, 3, 7, 11/*mean (0.1074), correlation (0.413224)*/,
                    5, -13, 6, 10/*mean (0.109256), correlation (0.408646)*/,
                    2, -12, 2, 3/*mean (0.131691), correlation (0.416076)*/,
                    3, 8, 4, -6/*mean (0.165081), correlation (0.417569)*/,
                    2, 6, 12, -13/*mean (0.171874), correlation (0.408471)*/,
                    9, -12, 10, 3/*mean (0.175146), correlation (0.41296)*/,
                    -8, 4, -7, 9/*mean (0.183682), correlation (0.402956)*/,
                    -11, 12, -4, -6/*mean (0.184672), correlation (0.416125)*/,
                    1, 12, 2, -8/*mean (0.191487), correlation (0.386696)*/,
                    6, -9, 7, -4/*mean (0.192668), correlation (0.394771)*/,
                    2, 3, 3, -2/*mean (0.200157), correlation (0.408303)*/,
                    6, 3, 11, 0/*mean (0.204588), correlation (0.411762)*/,
                    3, -3, 8, -8/*mean (0.205904), correlation (0.416294)*/,
                    7, 8, 9, 3/*mean (0.213237), correlation (0.409306)*/,
                    -11, -5, -6, -4/*mean (0.243444), correlation (0.395069)*/,
                    -10, 11, -5, 10/*mean (0.247672), correlation (0.413392)*/,
                    -5, -8, -3, 12/*mean (0.24774), correlation (0.411416)*/,
                    -10, 5, -9, 0/*mean (0.00213675), correlation (0.454003)*/,
                    8, -1, 12, -6/*mean (0.0293635), correlation (0.455368)*/,
                    4, -6, 6, -11/*mean (0.0404971), correlation (0.457393)*/,
                    -10, 12, -8, 7/*mean (0.0481107), correlation (0.448364)*/,
                    4, -2, 6, 7/*mean (0.050641), correlation (0.455019)*/,
                    -2, 0, -2, 12/*mean (0.0525978), correlation (0.44338)*/,
                    -5, -8, -5, 2/*mean (0.0629667), correlation (0.457096)*/,
                    7, -6, 10, 12/*mean (0.0653846), correlation (0.445623)*/,
                    -9, -13, -8, -8/*mean (0.0858749), correlation (0.449789)*/,
                    -5, -13, -5, -2/*mean (0.122402), correlation (0.450201)*/,
                    8, -8, 9, -13/*mean (0.125416), correlation (0.453224)*/,
                    -9, -11, -9, 0/*mean (0.130128), correlation (0.458724)*/,
                    1, -8, 1, -2/*mean (0.132467), correlation (0.440133)*/,
                    7, -4, 9, 1/*mean (0.132692), correlation (0.454)*/,
                    -2, 1, -1, -4/*mean (0.135695), correlation (0.455739)*/,
                    11, -6, 12, -11/*mean (0.142904), correlation (0.446114)*/,
                    -12, -9, -6, 4/*mean (0.146165), correlation (0.451473)*/,
                    3, 7, 7, 12/*mean (0.147627), correlation (0.456643)*/,
                    5, 5, 10, 8/*mean (0.152901), correlation (0.455036)*/,
                    0, -4, 2, 8/*mean (0.167083), correlation (0.459315)*/,
                    -9, 12, -5, -13/*mean (0.173234), correlation (0.454706)*/,
                    0, 7, 2, 12/*mean (0.18312), correlation (0.433855)*/,
                    -1, 2, 1, 7/*mean (0.185504), correlation (0.443838)*/,
                    5, 11, 7, -9/*mean (0.185706), correlation (0.451123)*/,
                    3, 5, 6, -8/*mean (0.188968), correlation (0.455808)*/,
                    -13, -4, -8, 9/*mean (0.191667), correlation (0.459128)*/,
                    -5, 9, -3, -3/*mean (0.193196), correlation (0.458364)*/,
                    -4, -7, -3, -12/*mean (0.196536), correlation (0.455782)*/,
                    6, 5, 8, 0/*mean (0.1972), correlation (0.450481)*/,
                    -7, 6, -6, 12/*mean (0.199438), correlation (0.458156)*/,
                    -13, 6, -5, -2/*mean (0.211224), correlation (0.449548)*/,
                    1, -10, 3, 10/*mean (0.211718), correlation (0.440606)*/,
                    4, 1, 8, -4/*mean (0.213034), correlation (0.443177)*/,
                    -2, -2, 2, -13/*mean (0.234334), correlation (0.455304)*/,
                    2, -12, 12, 12/*mean (0.235684), correlation (0.443436)*/,
                    -2, -13, 0, -6/*mean (0.237674), correlation (0.452525)*/,
                    4, 1, 9, 3/*mean (0.23962), correlation (0.444824)*/,
                    -6, -10, -3, -5/*mean (0.248459), correlation (0.439621)*/,
                    -3, -13, -1, 1/*mean (0.249505), correlation (0.456666)*/,
                    7, 5, 12, -11/*mean (0.00119208), correlation (0.495466)*/,
                    4, -2, 5, -7/*mean (0.00372245), correlation (0.484214)*/,
                    -13, 9, -9, -5/*mean (0.00741116), correlation (0.499854)*/,
                    7, 1, 8, 6/*mean (0.0208952), correlation (0.499773)*/,
                    7, -8, 7, 6/*mean (0.0220085), correlation (0.501609)*/,
                    -7, -4, -7, 1/*mean (0.0233806), correlation (0.496568)*/,
                    -8, 11, -7, -8/*mean (0.0236505), correlation (0.489719)*/,
                    -13, 6, -12, -8/*mean (0.0268781), correlation (0.503487)*/,
                    2, 4, 3, 9/*mean (0.0323324), correlation (0.501938)*/,
                    10, -5, 12, 3/*mean (0.0399235), correlation (0.494029)*/,
                    -6, -5, -6, 7/*mean (0.0420153), correlation (0.486579)*/,
                    8, -3, 9, -8/*mean (0.0548021), correlation (0.484237)*/,
                    2, -12, 2, 8/*mean (0.0616622), correlation (0.496642)*/,
                    -11, -2, -10, 3/*mean (0.0627755), correlation (0.498563)*/,
                    -12, -13, -7, -9/*mean (0.0829622), correlation (0.495491)*/,
                    -11, 0, -10, -5/*mean (0.0843342), correlation (0.487146)*/,
                    5, -3, 11, 8/*mean (0.0929937), correlation (0.502315)*/,
                    -2, -13, -1, 12/*mean (0.113327), correlation (0.48941)*/,
                    -1, -8, 0, 9/*mean (0.132119), correlation (0.467268)*/,
                    -13, -11, -12, -5/*mean (0.136269), correlation (0.498771)*/,
                    -10, -2, -10, 11/*mean (0.142173), correlation (0.498714)*/,
                    -3, 9, -2, -13/*mean (0.144141), correlation (0.491973)*/,
                    2, -3, 3, 2/*mean (0.14892), correlation (0.500782)*/,
                    -9, -13, -4, 0/*mean (0.150371), correlation (0.498211)*/,
                    -4, 6, -3, -10/*mean (0.152159), correlation (0.495547)*/,
                    -4, 12, -2, -7/*mean (0.156152), correlation (0.496925)*/,
                    -6, -11, -4, 9/*mean (0.15749), correlation (0.499222)*/,
                    6, -3, 6, 11/*mean (0.159211), correlation (0.503821)*/,
                    -13, 11, -5, 5/*mean (0.162427), correlation (0.501907)*/,
                    11, 11, 12, 6/*mean (0.16652), correlation (0.497632)*/,
                    7, -5, 12, -2/*mean (0.169141), correlation (0.484474)*/,
                    -1, 12, 0, 7/*mean (0.169456), correlation (0.495339)*/,
                    -4, -8, -3, -2/*mean (0.171457), correlation (0.487251)*/,
                    -7, 1, -6, 7/*mean (0.175), correlation (0.500024)*/,
                    -13, -12, -8, -13/*mean (0.175866), correlation (0.497523)*/,
                    -7, -2, -6, -8/*mean (0.178273), correlation (0.501854)*/,
                    -8, 5, -6, -9/*mean (0.181107), correlation (0.494888)*/,
                    -5, -1, -4, 5/*mean (0.190227), correlation (0.482557)*/,
                    -13, 7, -8, 10/*mean (0.196739), correlation (0.496503)*/,
                    1, 5, 5, -13/*mean (0.19973), correlation (0.499759)*/,
                    1, 0, 10, -13/*mean (0.204465), correlation (0.49873)*/,
                    9, 12, 10, -1/*mean (0.209334), correlation (0.49063)*/,
                    5, -8, 10, -9/*mean (0.211134), correlation (0.503011)*/,
                    -1, 11, 1, -13/*mean (0.212), correlation (0.499414)*/,
                    -9, -3, -6, 2/*mean (0.212168), correlation (0.480739)*/,
                    -1, -10, 1, 12/*mean (0.212731), correlation (0.502523)*/,
                    -13, 1, -8, -10/*mean (0.21327), correlation (0.489786)*/,
                    8, -11, 10, -6/*mean (0.214159), correlation (0.488246)*/,
                    2, -13, 3, -6/*mean (0.216993), correlation (0.50287)*/,
                    7, -13, 12, -9/*mean (0.223639), correlation (0.470502)*/,
                    -10, -10, -5, -7/*mean (0.224089), correlation (0.500852)*/,
                    -10, -8, -8, -13/*mean (0.228666), correlation (0.502629)*/,
                    4, -6, 8, 5/*mean (0.22906), correlation (0.498305)*/,
                    3, 12, 8, -13/*mean (0.233378), correlation (0.503825)*/,
                    -4, 2, -3, -3/*mean (0.234323), correlation (0.476692)*/,
                    5, -13, 10, -12/*mean (0.236392), correlation (0.475462)*/,
                    4, -13, 5, -1/*mean (0.236842), correlation (0.504132)*/,
                    -9, 9, -4, 3/*mean (0.236977), correlation (0.497739)*/,
                    0, 3, 3, -9/*mean (0.24314), correlation (0.499398)*/,
                    -12, 1, -6, 1/*mean (0.243297), correlation (0.489447)*/,
                    3, 2, 4, -8/*mean (0.00155196), correlation (0.553496)*/,
                    -10, -10, -10, 9/*mean (0.00239541), correlation (0.54297)*/,
                    8, -13, 12, 12/*mean (0.0034413), correlation (0.544361)*/,
                    -8, -12, -6, -5/*mean (0.003565), correlation (0.551225)*/,
                    2, 2, 3, 7/*mean (0.00835583), correlation (0.55285)*/,
                    10, 6, 11, -8/*mean (0.00885065), correlation (0.540913)*/,
                    6, 8, 8, -12/*mean (0.0101552), correlation (0.551085)*/,
                    -7, 10, -6, 5/*mean (0.0102227), correlation (0.533635)*/,
                    -3, -9, -3, 9/*mean (0.0110211), correlation (0.543121)*/,
                    -1, -13, -1, 5/*mean (0.0113473), correlation (0.550173)*/,
                    -3, -7, -3, 4/*mean (0.0140913), correlation (0.554774)*/,
                    -8, -2, -8, 3/*mean (0.017049), correlation (0.55461)*/,
                    4, 2, 12, 12/*mean (0.01778), correlation (0.546921)*/,
                    2, -5, 3, 11/*mean (0.0224022), correlation (0.549667)*/,
                    6, -9, 11, -13/*mean (0.029161), correlation (0.546295)*/,
                    3, -1, 7, 12/*mean (0.0303081), correlation (0.548599)*/,
                    11, -1, 12, 4/*mean (0.0355151), correlation (0.523943)*/,
                    -3, 0, -3, 6/*mean (0.0417904), correlation (0.543395)*/,
                    4, -11, 4, 12/*mean (0.0487292), correlation (0.542818)*/,
                    2, -4, 2, 1/*mean (0.0575124), correlation (0.554888)*/,
                    -10, -6, -8, 1/*mean (0.0594242), correlation (0.544026)*/,
                    -13, 7, -11, 1/*mean (0.0597391), correlation (0.550524)*/,
                    -13, 12, -11, -13/*mean (0.0608974), correlation (0.55383)*/,
                    6, 0, 11, -13/*mean (0.065126), correlation (0.552006)*/,
                    0, -1, 1, 4/*mean (0.074224), correlation (0.546372)*/,
                    -13, 3, -9, -2/*mean (0.0808592), correlation (0.554875)*/,
                    -9, 8, -6, -3/*mean (0.0883378), correlation (0.551178)*/,
                    -13, -6, -8, -2/*mean (0.0901035), correlation (0.548446)*/,
                    5, -9, 8, 10/*mean (0.0949843), correlation (0.554694)*/,
                    2, 7, 3, -9/*mean (0.0994152), correlation (0.550979)*/,
                    -1, -6, -1, -1/*mean (0.10045), correlation (0.552714)*/,
                    9, 5, 11, -2/*mean (0.100686), correlation (0.552594)*/,
                    11, -3, 12, -8/*mean (0.101091), correlation (0.532394)*/,
                    3, 0, 3, 5/*mean (0.101147), correlation (0.525576)*/,
                    -1, 4, 0, 10/*mean (0.105263), correlation (0.531498)*/,
                    3, -6, 4, 5/*mean (0.110785), correlation (0.540491)*/,
                    -13, 0, -10, 5/*mean (0.112798), correlation (0.536582)*/,
                    5, 8, 12, 11/*mean (0.114181), correlation (0.555793)*/,
                    8, 9, 9, -6/*mean (0.117431), correlation (0.553763)*/,
                    7, -4, 8, -12/*mean (0.118522), correlation (0.553452)*/,
                    -10, 4, -10, 9/*mean (0.12094), correlation (0.554785)*/,
                    7, 3, 12, 4/*mean (0.122582), correlation (0.555825)*/,
                    9, -7, 10, -2/*mean (0.124978), correlation (0.549846)*/,
                    7, 0, 12, -2/*mean (0.127002), correlation (0.537452)*/,
                    -1, -6, 0, -11/*mean (0.127148), correlation (0.547401)*/
            };

    ORBExtractor::ORBExtractor(const KeyPointMethod &method) : mMethod(method) {
        mnFeaturesPerLevel.resize(setting::numPyramid);
        float factor = 1.0f / setting::scaleFactors[1];
        float nDesiredFeaturesPerScale = setting::extractFeatures * (1 - factor) /
                                         (1 - (float) pow((double) factor, (double) setting::numPyramid));
        int sumFeatures = 0;
	
        for (int level = 0; level < setting::numPyramid - 1; level++) {
            mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
            sumFeatures += mnFeaturesPerLevel[level];
            nDesiredFeaturesPerScale *= factor;
        }
        mnFeaturesPerLevel[setting::numPyramid - 1] = std::max(int(setting::numPyramid - sumFeatures), int(0));

        const int npoints = 512;
        const Point *pattern0 = (const Point *) bit_pattern_31_;
        std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

        // This is for orientation
        // pre-compute the end of a row in a circular patch
        umax.resize(setting::HALF_PATCH_SIZE + 1);

        int v, v0, vmax = cvFloor(setting::HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
        int vmin = cvCeil(setting::HALF_PATCH_SIZE * sqrt(2.f) / 2);
        const double hp2 = setting::HALF_PATCH_SIZE * setting::HALF_PATCH_SIZE;
        for (v = 0; v <= vmax; ++v)
            umax[v] = cvRound(sqrt(hp2 - v * v));

        // Make sure we are symmetric
        for (v = setting::HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v) {
            while (umax[v0] == umax[v0 + 1])
                ++v0;
            umax[v] = v0;
            ++v0;
        }
        mnGridSize = -1;
    }


    float IC_Angle(const Mat &image, const Vector2f &pt, const vector<int> &u_max) {
        int m_01 = 0, m_10 = 0;
        const uchar *center = &image.at<uchar>(cvRound(pt[1]), cvRound(pt[0]));

        // Treat the center line differently, v=0
        for (int u = -setting::HALF_PATCH_SIZE; u <= setting::HALF_PATCH_SIZE; ++u)
            m_10 += u * center[u];

        // Go line by line in the circuI853lar patch
        int step = (int) image.step1();
        for (int v = 1; v <= setting::HALF_PATCH_SIZE; ++v) {
            // Proceed over the two lines
            int v_sum = 0;
            int d = u_max[v];
            for (int u = -d; u <= d; ++u) {
                int val_plus = center[u + v * step], val_minus = center[u - v * step];
                v_sum += (val_plus - val_minus);
                m_10 += u * (val_plus + val_minus);
            }
            m_01 += v * v_sum;
        }

        return fastAtan2((float) m_01, (float) m_10);
    }

    const float factorPI = (float) (CV_PI / 180.f);

    void computeOrbDescriptor(
            const Vector2f &kpt,
            const float &kpangle,
            const Mat &img, const Point *pattern,
            uchar *desc
    ) {
        float angle = (float) kpangle * factorPI;
        float a = (float) cos(angle), b = (float) sin(angle);

        const uchar *center = &img.at<uchar>(cvRound(kpt[1]), cvRound(kpt[0]));
        const int step = (int) img.step;

        // TODO this can be accelerated by SSE
#define GET_VALUE(idx) \
        center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
               cvRound(pattern[idx].x*a - pattern[idx].y*b)]
        for (int i = 0; i < 32; ++i, pattern += 16) {
            int t0, t1, val;
            t0 = GET_VALUE(0);
            t1 = GET_VALUE(1);
            val = t0 < t1;
            t0 = GET_VALUE(2);
            t1 = GET_VALUE(3);
            val |= (t0 < t1) << 1;
            t0 = GET_VALUE(4);
            t1 = GET_VALUE(5);
            val |= (t0 < t1) << 2;
            t0 = GET_VALUE(6);
            t1 = GET_VALUE(7);
            val |= (t0 < t1) << 3;
            t0 = GET_VALUE(8);
            t1 = GET_VALUE(9);
            val |= (t0 < t1) << 4;
            t0 = GET_VALUE(10);
            t1 = GET_VALUE(11);
            val |= (t0 < t1) << 5;
            t0 = GET_VALUE(12);
            t1 = GET_VALUE(13);
            val |= (t0 < t1) << 6;
            t0 = GET_VALUE(14);
            t1 = GET_VALUE(15);
            val |= (t0 < t1) << 7;

            desc[i] = (uchar) val;
        }
#undef GET_VALUE
    }

    void computeOrientation(const Mat &image, vector<shared_ptr<Feature>> &keypoints, const vector<int> &umax) {
        for (shared_ptr<Feature> feature: keypoints) {
            feature->mAngle = IC_Angle(image, feature->mPixel, umax);
        }
    }

    void computeDescriptors(const Mat &image, vector<shared_ptr<Feature>> &keypoints, const vector<Point> &pattern) {
        for (size_t i = 0; i < keypoints.size(); i++)
            computeOrbDescriptor(keypoints[i]->mPixel, keypoints[i]->mAngle, image, &pattern[0], keypoints[i]->mDesc);
    }

    void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4) {
        const int halfX = ceil(static_cast<float>(UR[0] - UL[0]) / 2);
        const int halfY = ceil(static_cast<float>(BR[1] - UL[1]) / 2);

        //Define boundaries of childs
        n1.UL = UL;
        n1.UR = Vector2i(UL[0] + halfX, UL[1]);
        n1.BL = Vector2i(UL[0], UL[1] + halfY);
        n1.BR = Vector2i(UL[0] + halfX, UL[1] + halfY);
        n1.vKeys.reserve(vKeys.size());

        n2.UL = n1.UR;
        n2.UR = UR;
        n2.BL = n1.BR;
        n2.BR = Vector2i(UR[0], UL[1] + halfY);
        n2.vKeys.reserve(vKeys.size());

        n3.UL = n1.BL;
        n3.UR = n1.BR;
        n3.BL = BL;
        n3.BR = Vector2i(n1.BR[0], BL[1]);
        n3.vKeys.reserve(vKeys.size());

        n4.UL = n3.UR;
        n4.UR = n2.BR;
        n4.BL = n3.BR;
        n4.BR = BR;
        n4.vKeys.reserve(vKeys.size());

        //Associate points to childs
        for (size_t i = 0; i < vKeys.size(); i++) {
            shared_ptr<Feature> kp = vKeys[i];
            if (kp->mPixel[0] < n1.UR[0]) {
                if (kp->mPixel[1] < n1.BR[1])
                    n1.vKeys.push_back(kp);
                else
                    n3.vKeys.push_back(kp);
            } else if (kp->mPixel[1] < n1.BR[1])
                n2.vKeys.push_back(kp);
            else
                n4.vKeys.push_back(kp);
        }

        if (n1.vKeys.size() == 1)
            n1.bNoMore = true;
        if (n2.vKeys.size() == 1)
            n2.bNoMore = true;
        if (n3.vKeys.size() == 1)
            n3.bNoMore = true;
        if (n4.vKeys.size() == 1)
            n4.bNoMore = true;
    }

    void ORBExtractor::Detect(shared_ptr<Frame> frame, bool leftEye, bool computeRotAndDesc) {
        mpFrame = frame;
        mbComputeRotAndDesc = computeRotAndDesc;

        vector<vector<shared_ptr<Feature>>> allKeypoints;
        allKeypoints.resize(setting::numPyramid);

        if (mMethod == FAST_MULTI_LEVEL) {
            // Multi level FAST
            if (leftEye) {
                ComputeKeyPointsFast(allKeypoints, frame->mPyramidLeft);
            } else {
                ComputeKeyPointsFast(allKeypoints, frame->mPyramidRight);
            }
        } else if (mMethod == FAST_SINGLE_LEVEL) {
            // SINGLE LEVEL
            vector<shared_ptr<Feature> > single_level_features;
            LOG(INFO) << "Calling fast single level" << endl;
            if (leftEye) {
                ComputeKeyPointsFastSingleLevel(single_level_features, frame->mImLeft);
            } else {
                ComputeKeyPointsFastSingleLevel(single_level_features, frame->mImRight);
            }
            allKeypoints[0] = single_level_features;
        } else if (mMethod == ORB_SLAM2) {
            // From ORB-SLAM2
            if (leftEye) {
                ComputeKeyPointsOctTree(allKeypoints, frame->mPyramidLeft);
            } else {
                ComputeKeyPointsOctTree(allKeypoints, frame->mPyramidRight);
            }
        } else if (mMethod == OPENCV_ORB) {
            // From OpenCV
            if (leftEye) {
                ComputeKeyPointsORBOpenCV(allKeypoints, frame->mPyramidLeft);
            } else {
                ComputeKeyPointsORBOpenCV(allKeypoints, frame->mPyramidRight);
            }
        } else if (mMethod == OPENCV_GFTT) {
            if (leftEye) {
                ComputeKeyPointsGFTT(allKeypoints, frame->mImLeft);
            } else {
                ComputeKeyPointsGFTT(allKeypoints, frame->mImRight);
            }
        }

        if (mbComputeRotAndDesc) {
            // 计算 bag-of-words
            vector<cv::Mat> pyramid_blured;
            pyramid_blured.resize(setting::numPyramid);
            for (size_t i = 0; i < setting::numPyramid; i++) {
                if (leftEye)
                    pyramid_blured[i] = frame->mPyramidLeft[i].clone();
                else
                    pyramid_blured[i] = frame->mPyramidRight[i].clone();

                GaussianBlur(pyramid_blured[i], pyramid_blured[i], Size(7, 7), 2, 2, BORDER_REFLECT_101);
            }

            for (size_t level = 0; level < setting::numPyramid; level++) {
                vector<shared_ptr<Feature>> &features = allKeypoints[level];
                if (features.empty())
                    continue;
                computeDescriptors(pyramid_blured[level], features, pattern);
                for (auto f: features) {
                    f->mPixel *= setting::scaleFactors[level];
                    if (leftEye) {
                        frame->mFeaturesLeft.push_back(f);
                    } else {
                        frame->mFeaturesRight.push_back(f);
                    }
                }
            }
        } else {
            for (size_t level = 0; level < setting::numPyramid; level++) {
                vector<shared_ptr<Feature>> &features = allKeypoints[level];
                if (features.empty())
                    continue;
                for (auto f: features) {
                    f->mPixel *= setting::scaleFactors[level];
                    if (leftEye) {
                        frame->mFeaturesLeft.push_back(f);
                    } else {
                        frame->mFeaturesRight.push_back(f);
                    }
                }
            }
        }
    }

    void ORBExtractor::ComputeKeyPointsFast(
            std::vector<std::vector<shared_ptr<Feature>>> &allKeypoints,
            const std::vector<cv::Mat> &pyramid) {

        // 网格化的fast
        const int mnCellSize = 10;   // fixed grid size
        int mnGridCols = setting::imageWidth / mnCellSize;
        int mnGridRows = setting::imageHeight / mnCellSize;
        std::vector<bool> mvbGridOccupancy;
        vector<shared_ptr<Feature>> featureGrid;
        featureGrid.resize(mnGridCols * mnGridRows, nullptr);

        allKeypoints.resize(setting::numPyramid);
        int numPoints = 0;

        for (size_t level = 0; level < setting::numPyramid; level++) {
            double scale = setting::scaleFactors[level];

            allKeypoints[level].reserve(setting::extractFeatures);
            vector<fast::fast_xy> fast_corners;

            // 去掉边界
            int boarder = setting::boarder;
            const uchar *data_start = pyramid[level].ptr<uchar>(boarder) + boarder;

#ifdef __SSE2__
            // fast sse2
            fast::fast_corner_detect_10_sse2(
                    (fast::fast_byte *) data_start, pyramid[level].cols - boarder * 2,
                    pyramid[level].rows - boarder * 2,
                    pyramid[level].cols, setting::initTHFAST, fast_corners);
#else
            fast::fast_corner_detect_10 (
            ( fast::fast_byte* ) data_start, pyramid[level].cols-boarder*2,
            pyramid[level].rows-boarder*2,
            pyramid[level].cols, setting::initTHFAST, fast_corners );
#endif
            // nonmax
            vector<int> scores, nm_corners;
            fast::fast_corner_score_10(
                    (fast::fast_byte *) data_start,
                    pyramid[level].cols, fast_corners, setting::initTHFAST, scores);

            fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);
            allKeypoints[level].reserve(nm_corners.size());

            for (int nm: nm_corners) {
                fast::fast_xy &xy = fast_corners[nm];

                xy.x += boarder;
                xy.y += boarder;

                const int gy = static_cast<int> ((xy.y * scale) / mnCellSize );
                const int gx = static_cast<int> ((xy.x * scale) / mnCellSize );
                const size_t k = gy * mnGridCols + gx;

                assert(k >= 0 && k < featureGrid.size());
                shared_ptr<Feature> feat = featureGrid[k];
                float score = ShiTomasiScore(pyramid[level], xy.x, xy.y);
                if (score > setting::minShiTomasiScore && (feat == nullptr || feat->mScore < score)) {
                    // create a new feature
                    shared_ptr<Feature> feature = make_shared<Feature>();
                    feature->mPixel = Vector2f(xy.x, xy.y);
                    feature->mScore = score;
                    feature->mLevel = level;
                    feature->mAngle = IC_Angle(pyramid[feature->mLevel], feature->mPixel, umax);
                    featureGrid[k] = feature;
                }
            }

            for (shared_ptr<Feature> feat: featureGrid) {
                if (feat && feat->mScore > 20) {
                    allKeypoints[level].push_back(feat);
                }
            }

            numPoints += allKeypoints[level].size();
            if (numPoints > setting::extractFeatures)
                break;
        }
    }

    void ORBExtractor::ComputeKeyPointsFastSingleLevel(std::vector<shared_ptr<Feature>> &allKeypoints,
                                                       const cv::Mat &image) {
        mpFrame->AssignFeaturesToGrid();
        allKeypoints.reserve(setting::extractFeatures * 2);
        int cnt = 0;   // 特征数
        // 试图在每个没有特征的网格中寻找一个特征
        for (int i = 1; i < setting::FRAME_GRID_ROWS - 1; i++) {
            for (int j = 1; j < setting::FRAME_GRID_COLS - 1; j++) {
                if (mpFrame->mGrid[i * setting::FRAME_GRID_COLS + j].empty()) {

                    // 尝试在此网格中提取一个特征
                    const uchar *data = image.ptr<uchar>(i * setting::FRAME_GRID_SIZE) + j * setting::FRAME_GRID_SIZE;
                    vector<fast::fast_xy> fast_corners;
#ifdef __SSE2__
                    fast::fast_corner_detect_10_sse2(data, setting::FRAME_GRID_SIZE, setting::FRAME_GRID_SIZE,
                                                     setting::imageWidth, setting::initTHFAST, fast_corners);
#else
                    fast::fast_corner_detect_10(data, setting::FRAME_GRID_SIZE, setting::FRAME_GRID_SIZE, setting::imageWidth, setting::initTHFAST, fast_corners);
#endif
                    if (fast_corners.empty()) {
                        // try lower threshold
#ifdef __SSE2__
                        fast::fast_corner_detect_10_sse2(data, setting::FRAME_GRID_SIZE, setting::FRAME_GRID_SIZE,
                                                         setting::imageWidth, setting::minTHFAST, fast_corners);
#else
                        fast::fast_corner_detect_10(data, setting::FRAME_GRID_SIZE, setting::FRAME_GRID_SIZE, setting::imageWidth, setting::minTHFAST, fast_corners);
#endif
                    }

                    if (fast_corners.empty())
                        continue;

                    // find the best one and insert as a feature
                    int x_start = j * setting::FRAME_GRID_SIZE;
                    int y_start = i * setting::FRAME_GRID_SIZE;

                    // sort the corners according to shi-tomasi score
                    vector<pair<fast::fast_xy, float> > corner_score;
                    int idxBest = 0;
                    float scoreBest = -1;
                    for (int k = 0; k < fast_corners.size(); k++) {
                        fast::fast_xy &xy = fast_corners[k];
                        xy.x += x_start;
                        xy.y += y_start;

                        if (xy.x < setting::boarder || xy.y < setting::boarder ||
                            xy.x >= setting::imageWidth - setting::boarder ||
                            xy.y >= setting::imageHeight - setting::boarder) {
                            // 太边缘不便于计划描述子
                            continue;
                        }

                        float score = ShiTomasiScore(image, xy.x, xy.y);
                        if (/* score > setting::minShiTomasiScore && */ score > scoreBest) {
                            scoreBest = score;
                            idxBest = k;
                        }
                    }

                    if (scoreBest < 0)
                        continue;

                    // 按score降序
                    fast::fast_xy &best = fast_corners[idxBest];
                    shared_ptr<Feature> feature(new Feature());
                    feature->mPixel = Vector2f(best.x, best.y);
                    if (mbComputeRotAndDesc)
                        feature->mAngle = IC_Angle(image, feature->mPixel, umax);
                    allKeypoints.push_back(feature);
                }
            }
        }

    }

    vector<shared_ptr<Feature>> ORBExtractor::DistributeOctTree(
            const vector<shared_ptr<Feature>> &vToDistributeKeys, const int &minX,
            const int &maxX, const int &minY, const int &maxY,
            const int &N, const int &level) {

        // Compute how many initial nodes
        const int nIni = round(static_cast<float>(maxX - minX) / (maxY - minY));

        const float hX = static_cast<float>(maxX - minX) / nIni;

        list <ExtractorNode, Eigen::aligned_allocator<ExtractorNode>> lNodes;

        vector<ExtractorNode *> vpIniNodes;
        vpIniNodes.resize(nIni);

        for (int i = 0; i < nIni; i++) {
            ExtractorNode ni;
            ni.UL = Vector2i(hX * static_cast<float>(i), 0);
            ni.UR = Vector2i(hX * static_cast<float>(i + 1), 0);
            ni.BL = Vector2i(ni.UL[0], maxY - minY);
            ni.BR = Vector2i(ni.UR[0], maxY - minY);
            ni.vKeys.reserve(vToDistributeKeys.size());

            lNodes.push_back(ni);
            vpIniNodes[i] = &lNodes.back();
        }

        //Associate points to childs
        for (size_t i = 0; i < vToDistributeKeys.size(); i++) {
            shared_ptr<Feature> kp = vToDistributeKeys[i];
            vpIniNodes[kp->mPixel[0] / hX]->vKeys.push_back(kp);
        }

        auto lit = lNodes.begin();

        while (lit != lNodes.end()) {
            if (lit->vKeys.size() == 1) {
                lit->bNoMore = true;
                lit++;
            } else if (lit->vKeys.empty())
                lit = lNodes.erase(lit);
            else
                lit++;
        }

        bool bFinish = false;

        int iteration = 0;

        vector<pair<int, ExtractorNode *> > vSizeAndPointerToNode;
        vSizeAndPointerToNode.reserve(lNodes.size() * 4);

        while (!bFinish) {
            iteration++;

            int prevSize = lNodes.size();

            lit = lNodes.begin();

            int nToExpand = 0;

            vSizeAndPointerToNode.clear();

            while (lit != lNodes.end()) {
                if (lit->bNoMore) {
                    // If node only contains one point do not subdivide and continue
                    lit++;
                    continue;
                } else {
                    // If more than one point, subdivide
                    ExtractorNode n1, n2, n3, n4;
                    lit->DivideNode(n1, n2, n3, n4);

                    // Add childs if they contain points
                    if (n1.vKeys.size() > 0) {
                        lNodes.push_front(n1);
                        if (n1.vKeys.size() > 1) {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n2.vKeys.size() > 0) {
                        lNodes.push_front(n2);
                        if (n2.vKeys.size() > 1) {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n3.vKeys.size() > 0) {
                        lNodes.push_front(n3);
                        if (n3.vKeys.size() > 1) {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n4.vKeys.size() > 0) {
                        lNodes.push_front(n4);
                        if (n4.vKeys.size() > 1) {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lit = lNodes.erase(lit);
                    continue;
                }
            }

            // Finish if there are more nodes than required features
            // or all nodes contain just one point
            if ((int) lNodes.size() >= N || (int) lNodes.size() == prevSize) {
                bFinish = true;
            } else if (((int) lNodes.size() + nToExpand * 3) > N) {

                while (!bFinish) {

                    prevSize = lNodes.size();

                    vector<pair<int, ExtractorNode *> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                    vSizeAndPointerToNode.clear();

                    sort(vPrevSizeAndPointerToNode.begin(), vPrevSizeAndPointerToNode.end());
                    for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; j--) {
                        ExtractorNode n1, n2, n3, n4;
                        vPrevSizeAndPointerToNode[j].second->DivideNode(n1, n2, n3, n4);

                        // Add childs if they contain points
                        if (n1.vKeys.size() > 0) {
                            lNodes.push_front(n1);
                            if (n1.vKeys.size() > 1) {
                                vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n2.vKeys.size() > 0) {
                            lNodes.push_front(n2);
                            if (n2.vKeys.size() > 1) {
                                vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n3.vKeys.size() > 0) {
                            lNodes.push_front(n3);
                            if (n3.vKeys.size() > 1) {
                                vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n4.vKeys.size() > 0) {
                            lNodes.push_front(n4);
                            if (n4.vKeys.size() > 1) {
                                vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }

                        lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                        if ((int) lNodes.size() >= N)
                            break;
                    }

                    if ((int) lNodes.size() >= N || (int) lNodes.size() == prevSize)
                        bFinish = true;

                }
            }
        }

        // Retain the best point in each node
        vector<shared_ptr<Feature>> vResultKeys;
        vResultKeys.reserve(setting::extractFeatures);
        for (auto lit = lNodes.begin(); lit != lNodes.end(); lit++) {
            vector<shared_ptr<Feature> > &vNodeKeys = lit->vKeys;
            shared_ptr<Feature> pKP = vNodeKeys[0];
            float maxResponse = pKP->mScore;
            for (size_t k = 1; k < vNodeKeys.size(); k++) {
                if (vNodeKeys[k]->mScore > maxResponse) {
                    pKP = vNodeKeys[k];
                    maxResponse = vNodeKeys[k]->mScore;
                }
            }

            vResultKeys.push_back(pKP);
        }

        return vResultKeys;
    }

    void ORBExtractor::ComputeKeyPointsOctTree(
            std::vector<std::vector<shared_ptr<Feature >>> &allKeypoints, vector<cv::Mat> &pyramid) {

        assert(mpFrame != nullptr);

        allKeypoints.resize(setting::numPyramid);

        const float W = 30;

        for (int level = 0; level < setting::numPyramid; ++level) {

            const int minBorderX = setting::EDGE_THRESHOLD - 3;
            const int minBorderY = minBorderX;
            const int maxBorderX = pyramid[level].cols - setting::EDGE_THRESHOLD + 3;
            const int maxBorderY = pyramid[level].rows - setting::EDGE_THRESHOLD + 3;

            vector<shared_ptr<Feature>> vToDistributeKeys;
            vToDistributeKeys.reserve(setting::extractFeatures * 10);

            const float width = (maxBorderX - minBorderX);
            const float height = (maxBorderY - minBorderY);

            const int nCols = width / W;
            const int nRows = height / W;
            const int wCell = ceil(float(width) / nCols);
            const int hCell = ceil(float(height) / nRows);

            for (int i = 0; i < nRows; i++) {
                const float iniY = minBorderY + i * hCell;
                float maxY = iniY + hCell + 6;

                if (iniY >= maxBorderY - 3)
                    continue;
                if (maxY > maxBorderY)
                    maxY = maxBorderY;

                for (int j = 0; j < nCols; j++) {
                    const float iniX = minBorderX + j * wCell;
                    float maxX = iniX + wCell + 6;
                    if (iniX >= maxBorderX - 6)
                        continue;
                    if (maxX > maxBorderX)
                        maxX = maxBorderX;

                    vector<cv::KeyPoint> vKeysCell;
                    FAST(pyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
                         vKeysCell, setting::initTHFAST, true);
                    if (vKeysCell.empty()) {
                        FAST(pyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
                             vKeysCell, setting::minTHFAST, true);
                    }

                    if (!vKeysCell.empty()) {
                        for (vector<cv::KeyPoint>::iterator vit = vKeysCell.begin(); vit != vKeysCell.end(); vit++) {
                            (*vit).pt.x += j * wCell;
                            (*vit).pt.y += i * hCell;

                            shared_ptr<Feature> feature = make_shared<Feature>();
                            feature->mPixel = Vector2f(vit->pt.x, vit->pt.y);
                            feature->mLevel = level;
                            feature->mScore = vit->response;
                            vToDistributeKeys.push_back(feature);
                        }
                    }
                }
            }

            auto &keypoints = allKeypoints[level];
            keypoints.reserve(setting::extractFeatures);

            keypoints = DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX,
                                          minBorderY, maxBorderY, mnFeaturesPerLevel[level], level);
            // Add border to coordinates and scale information
            for (auto feature: keypoints) {
                feature->mPixel[0] += minBorderX;
                feature->mPixel[1] += minBorderY;
            }
        }

        // compute orientations
        for (int level = 0; level < setting::numPyramid; ++level)
            computeOrientation(pyramid[level], allKeypoints[level], umax);

    }

    void ORBExtractor::ComputeKeyPointsORBOpenCV(std::vector<std::vector<shared_ptr<Feature>>> &allPoints,
                                                 const std::vector<cv::Mat> &pyramid) {

        LOG(ERROR) << "This function is not implemented!" << endl;
        /*
        Ptr<FeatureDetector> detector = ORB::create(setting::extractFeatures, setting::scaleFactors[0],
        3, setting::initTHFAST);

        // Ptr<FeatureDetector> detector = Feature2D::create("ORB");

        vector<cv::KeyPoint> kps;
        detector->detect(pyramid[0], kps);

        for (cv::KeyPoint &kp: kps) {
            shared_ptr<Feature> feat = make_shared<Feature>();
            Vector2f pixel(kp.pt.x, kp.pt.y);
            pixel = pixel * setting::invScaleFactors[kp.octave];
            float wl = setting::imageWidth * setting::invScaleFactors[kp.octave];
            float hl = setting::imageHeight * setting::invScaleFactors[kp.octave];

            if (pixel[0] < setting::boarder || pixel[1] < setting::boarder ||
                pixel[0] >= wl - setting::boarder || pixel[1] >= hl - setting::boarder) {
                LOG(INFO) << pixel.transpose() << ", level = " << kp.octave << endl;
                continue;
            }
            feat->mPixel = pixel;
            feat->mLevel = kp.octave;
            feat->mAngle = kp.angle;

            allPoints[kp.octave].push_back(feat);
        }
         */
    }

    void ORBExtractor::ComputeKeyPointsGFTT(std::vector<std::vector<shared_ptr<Feature>>> &allKeypoints,
                                            const cv::Mat &img) {
        // they always give points in level 0
        LOG(INFO) << "Calling GFTT" << endl;
        vector<cv::Point2f> pts;
        // create the mask, don't extract points in grid that already has a feature
        cv::Mat mask(setting::imageHeight, setting::imageWidth, CV_8U, cv::Scalar(0));
        for (size_t row = 1; row < setting::FRAME_GRID_ROWS - 1; row++)
            for (size_t col = 1; col < setting::FRAME_GRID_COLS - 1; col++) {
                cv::Rect rect(cv::Point2i(col * setting::FRAME_GRID_SIZE, row * setting::FRAME_GRID_SIZE),
                              cv::Size(setting::FRAME_GRID_SIZE, setting::FRAME_GRID_SIZE));
                if (mpFrame->mGrid[row * setting::FRAME_GRID_COLS + col].empty())
                    mask(rect) = cv::Scalar(255);
                else
                    mask(rect) = cv::Scalar(0);
            }


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        cv::goodFeaturesToTrack(img, pts, setting::extractFeatures, 0.01, setting::featureDistanceGFTT, mask);
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        double timeCost2 = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();
        LOG(INFO) << "cv::goodFeaturesToTrack cost time: " << timeCost2 << endl;

        if (pts.size() + mpFrame->mFeaturesLeft.size() < setting::extractFeatures / 2) {
            pts.clear();
            cv::goodFeaturesToTrack(img, pts, setting::extractFeatures, 0.01, setting::featureDistanceGFTT / 2, mask);
        }

        for (cv::Point2f &pt: pts) {
            if (pt.x < setting::boarder || pt.y < setting::boarder ||
                pt.x > setting::imageWidth - setting::boarder || pt.y > setting::imageHeight - setting::boarder)
                continue;

            // compute the grid location
            shared_ptr<Feature> feat(new Feature);
            feat->mPixel = Vector2f(pt.x, pt.y);
            allKeypoints[0].push_back(feat);
        }
    }
}
