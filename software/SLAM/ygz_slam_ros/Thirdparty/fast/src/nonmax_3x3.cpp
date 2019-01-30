 // This function is taken from libCVD. The only change is the return type which
 // returns the index of the maxima scores and not a new vector of scores!
 // Created on: Dec 18, 2012
 // Author: cforster

#include <fast/fast.h>
#include <vector>

namespace fast
{

//This function has been moved from fast_corner.cxx. Look there
//for the old ChangeLog.
// fast_nonmax_t is templated so you can have either of:
//      1: A vector of ImageRefs of the nonmax corners
//      2: A vector of <ImageRef, int> pairs of the corners and their scores.
void fast_nonmax_3x3(const std::vector<fast_xy>& corners, const std::vector<int>& scores, std::vector<int>& nonmax_corners)
{
    nonmax_corners.clear();
    nonmax_corners.reserve(corners.size());

    if(corners.size() < 1)
        return;


    // Find where each row begins
    // (the corners are output in raster scan order). A beginning of -1 signifies
    // that there are no corners on that row.
    int last_row = corners.back().y;
    vector<int> row_start(last_row + 1, -1);

    int prev_row = -1;
    for(unsigned int i=0; i< corners.size(); i++)
        if(corners[i].y != prev_row)
        {
                row_start[corners[i].y] = i;
                prev_row = corners[i].y;
        }


    //Point above points (roughly) to the pixel above the one of interest, if there
    //is a feature there.
    int point_above = 0;
    int point_below = 0;

    const int sz = (int)corners.size();

    for(int i=0; i < sz; i++)
    {
            int score = scores[i];
            fast_xy pos = corners[i];

            //Check left
            if(i > 0)
                //if(corners[i-1] == pos-ImageRef(1,0) && (scores[i-1] >= score))
                if(corners[i-1].x == pos.x-1 && corners[i-1].y == pos.y && scores[i-1] >= score)
                    continue;

            //Check right
            if(i < (sz - 1))
                //if(corners[i+1] == pos+ImageRef(1,0) &&  (scores[i+1] >= score))
                if(corners[i+1].x == pos.x+1 && corners[i+1].y == pos.y && scores[i+1] >= score)
                    continue;

            //Check above (if there is a valid row above)
            if(pos.y != 0 && row_start[pos.y - 1] != -1)
            {
                    //Make sure that current point_above is one
                    //row above.
                    if(corners[point_above].y < pos.y - 1)
                            point_above = row_start[pos.y-1];

                    //Make point_above point to the first of the pixels above the current point,
                    //if it exists.
                    for(; corners[point_above].y < pos.y && corners[point_above].x < pos.x - 1; point_above++)
                    {}


                    for(int i=point_above; corners[i].y < pos.y && corners[i].x <= pos.x + 1; i++)
                    {
                            int x = corners[i].x;
                            if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && (scores[i] >= score))
                                    goto cont;
                    }

            }

            //Check below (if there is anything below)
            if(pos.y != last_row && row_start[pos.y + 1] != -1 && point_below < sz) //Nothing below
            {
                    if(corners[point_below].y < pos.y + 1)
                            point_below = row_start[pos.y+1];

                    // Make point below point to one of the pixels belowthe current point, if it
                    // exists.
                    for(; point_below < sz && corners[point_below].y == pos.y+1 && corners[point_below].x < pos.x - 1; point_below++)
                    {}

                    for(int i=point_below; i < sz && corners[i].y == pos.y+1 && corners[i].x <= pos.x + 1; i++)
                    {
                            int x = corners[i].x;
                            if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && (scores[i] >= score))
                            goto cont;
                    }
            }

            nonmax_corners.push_back(i);

            cont:
                    ;
    }
}

}
