/**
* This file is part of ORB-SLAM2.
*/

#ifndef SLAMCOMPARATORS_H
#define SLAMCOMPARATORS_H

namespace ORB_SLAM2
{

class KeyFrame;
class MapPoint;

struct KFIdLess
{
    bool operator()(const KeyFrame* a, const KeyFrame* b) const;
};

struct MPIdLess
{
    bool operator()(const MapPoint* a, const MapPoint* b) const;
};

} //namespace ORB_SLAM

#endif // SLAMCOMPARATORS_H
