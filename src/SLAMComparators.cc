/**
* This file is part of ORB-SLAM2.
*/

#include "SLAMComparators.h"   // IWYU pragma: associated

#include "KeyFrame.h"
#include "MapPoint.h"

namespace ORB_SLAM2
{

bool KFIdLess::operator()(const KeyFrame* a, const KeyFrame* b) const
{
    if(a == b)
        return false;
    if(!a)
        return true;
    if(!b)
        return false;
    return a->mnId < b->mnId;
}

bool MPIdLess::operator()(const MapPoint* a, const MapPoint* b) const
{
    if(a == b)
        return false;
    if(!a)
        return true;
    if(!b)
        return false;
    return a->mnId < b->mnId;
}

} //namespace ORB_SLAM
