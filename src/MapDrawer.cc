/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"   // IWYU pragma: associated

#include <stddef.h>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <mutex>
#include <set>
#include <sstream>
#include <vector>

#include "Converter.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Map.h"


namespace ORB_SLAM2
{

namespace
{
constexpr float kDefaultKeyFrameSize = 0.1f;
constexpr float kDefaultKeyFrameLineWidth = 0.5f;
constexpr float kDefaultGraphLineWidth = 0.9f;
constexpr float kDefaultPointSize = 2.0f;
constexpr float kDefaultCameraSize = 0.20f;
constexpr float kDefaultCameraLineWidth = 3.0f;
}

MapDrawer::MapDrawer(Map* pMap, const string &, bool bEnableRerun) :
    mpMap(pMap), mRecordStream(rerun::RecordingStream("ORB_SLAM2")), mbEnableRerun(bEnableRerun),
    mbStaticSceneLogged(false), mKeyFrameSize(kDefaultKeyFrameSize),
    mKeyFrameLineWidth(kDefaultKeyFrameLineWidth), mGraphLineWidth(kDefaultGraphLineWidth),
    mPointSize(kDefaultPointSize), mCameraSize(kDefaultCameraSize),
    mCameraLineWidth(kDefaultCameraLineWidth)
{
    if(!mbEnableRerun)
        return;

    mRecordStream.spawn().exit_on_failure();
}

void MapDrawer::LogStaticScene()
{
    if(!mbEnableRerun || mbStaticSceneLogged)
        return;

    mRecordStream.log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);
    mRecordStream.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);
    const float w = mCameraSize;
    const float h = w * 0.75f;
    const float z = w * 0.6f;
    const vector<vector<rerun::Position3D>> cameraFrustum = {
        {{0.0f, 0.0f, 0.0f}, {w, h, z}},
        {{0.0f, 0.0f, 0.0f}, {w, -h, z}},
        {{0.0f, 0.0f, 0.0f}, {-w, -h, z}},
        {{0.0f, 0.0f, 0.0f}, {-w, h, z}},
        {{w, h, z}, {w, -h, z}},
        {{w, -h, z}, {-w, -h, z}},
        {{-w, -h, z}, {-w, h, z}},
        {{-w, h, z}, {w, h, z}},
    };
    mRecordStream.log_static(
        "world/current_camera/camera_frustum",
        rerun::LineStrips3D(cameraFrustum)
            .with_colors({rerun::Color(0, 255, 0)})
            .with_radii(mCameraLineWidth * 0.01f)
    );
    mbStaticSceneLogged = true;
}

void MapDrawer::DrawMapPoints()
{
    if(!mbEnableRerun)
        return;

    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*, MPIdLess> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    vector<rerun::Position3D> allPoints;
    vector<rerun::Position3D> refPoints;
    allPoints.reserve(vpMPs.size());
    refPoints.reserve(spRefMPs.size());

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        allPoints.emplace_back(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }

    for(auto sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        refPoints.emplace_back(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }

    mRecordStream.log(
        "world/map_points",
        rerun::Points3D(allPoints).with_colors({rerun::Color(0, 0, 0)}).with_radii(mPointSize * 0.01f)
    );
    mRecordStream.log(
        "world/reference_map_points",
        rerun::Points3D(refPoints).with_colors({rerun::Color(255, 0, 0)}).with_radii(mPointSize * 0.01f)
    );
}

void MapDrawer::DrawFrameImage(const cv::Mat &im)
{
    if(!mbEnableRerun || im.empty())
        return;

    cv::Mat rgb;
    if(im.channels() == 1)
        cv::cvtColor(im, rgb, cv::COLOR_GRAY2RGB);
    else if(im.channels() == 3)
        cv::cvtColor(im, rgb, cv::COLOR_BGR2RGB);
    else if(im.channels() == 4)
        cv::cvtColor(im, rgb, cv::COLOR_BGRA2RGBA);
    else
        return;

    if(!rgb.isContinuous())
        rgb = rgb.clone();

    const size_t byteCount = rgb.total() * rgb.elemSize();
    vector<uint8_t> bytes(rgb.data, rgb.data + byteCount);
    const rerun::WidthHeight resolution = {
        static_cast<uint32_t>(rgb.cols),
        static_cast<uint32_t>(rgb.rows)
    };

    if(rgb.channels() == 4)
    {
        mRecordStream.log(
            "frame/image",
            rerun::Image::from_rgba32(rerun::take_ownership(std::move(bytes)), resolution)
        );
    }
    else
    {
        mRecordStream.log(
            "frame/image",
            rerun::Image::from_rgb24(rerun::take_ownership(std::move(bytes)), resolution)
        );
    }
}

void MapDrawer::DrawGroundTruthTrajectory(const std::string &strTrajectoryPath)
{
    if(!mbEnableRerun)
        return;

    ifstream fTrajectory(strTrajectoryPath.c_str());
    if(!fTrajectory.is_open())
    {
        cerr << "MapDrawer: cannot open ground truth trajectory file " << strTrajectoryPath << endl;
        return;
    }

    vector<rerun::Position3D> trajectory;
    string line;
    while(getline(fTrajectory, line))
    {
        if(line.empty() || line[0] == '#')
            continue;

        stringstream ss(line);
        vector<double> values;
        double v;
        while(ss >> v)
            values.push_back(v);

        if(values.size() == 12)
        {
            // KITTI: row-major 3x4 pose matrix, translation at indices 3, 7, 11
            trajectory.emplace_back(
                static_cast<float>(values[3]),
                static_cast<float>(values[7]),
                static_cast<float>(values[11])
            );
        }
        else if(values.size() == 8)
        {
            // TUM: timestamp tx ty tz qx qy qz qw
            trajectory.emplace_back(
                static_cast<float>(values[1]),
                static_cast<float>(values[2]),
                static_cast<float>(values[3])
            );
        }
    }

    if(trajectory.empty())
    {
        cerr << "MapDrawer: no valid KITTI/TUM poses in " << strTrajectoryPath << endl;
        return;
    }

    mRecordStream.log_static(
        "world/ground_truth_trajectory",
        rerun::LineStrips3D(vector<vector<rerun::Position3D>>{trajectory})
            .with_colors({rerun::Color(255, 0, 255)})
            .with_radii(mGraphLineWidth * 0.01f)
    );
}

void MapDrawer::SetFrameId(const int frameId)
{
    if(!mbEnableRerun)
        return;

    LogStaticScene();
    mRecordStream.set_time_sequence("frame_id", frameId);
}

bool MapDrawer::IsRerunEnabled() const
{
    return mbEnableRerun;
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    if(!mbEnableRerun)
        return;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        vector<rerun::Position3D> keyframePoints;
        keyframePoints.reserve(vpKFs.size());

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Ow = pKF->GetCameraCenter();
            keyframePoints.emplace_back(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
        }

        mRecordStream.log(
            "world/keyframes",
            rerun::Points3D(keyframePoints)
                .with_colors({rerun::Color(0, 0, 255)})
                .with_radii(mKeyFrameSize * mKeyFrameLineWidth)
        );
    }

    if(bDrawGraph)
    {
        vector<vector<rerun::Position3D>> graphLines;

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            rerun::Position3D pOw(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    graphLines.push_back({
                        pOw,
                        rerun::Position3D(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2))
                    });
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                graphLines.push_back({
                    pOw,
                    rerun::Position3D(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2))
                });
            }

            // Loops
            auto sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(auto sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                graphLines.push_back({
                    pOw,
                    rerun::Position3D(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2))
                });
            }
        }

        mRecordStream.log(
            "world/keyframe_graph",
            rerun::LineStrips3D(graphLines)
                .with_colors({rerun::Color(0, 255, 0)})
                .with_radii(mGraphLineWidth * 0.001f)
        );
    }
}

void MapDrawer::DrawCurrentCamera()
{
    if(!mbEnableRerun)
        return;

    if(mCameraPose.empty())
        return;

    cv::Mat Rwc(3,3,CV_32F);
    cv::Mat twc(3,1,CV_32F);
    {
        unique_lock<mutex> lock(mMutexCamera);
        Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
        twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
    }

    const vector<float> q = Converter::toQuaternion(Rwc);
    mRecordStream.log(
        "world/current_camera",
        rerun::Transform3D::from_translation_rotation(
            rerun::components::Translation3D(
                rerun::Position3D(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2))
            ),
            rerun::Quaternion::from_xyzw(q[0], q[1], q[2], q[3])
        )
    );
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

} //namespace ORB_SLAM
