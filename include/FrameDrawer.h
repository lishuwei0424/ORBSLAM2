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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>


namespace ORB_SLAM2
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);

    // Update info from the last processed frame.  使用最新关键帧更行窗口 lishuwei 2018.11.24
    void Update(Tracking *pTracker);

    // Draw last processed frame.     //显示最新帧  lishuwei f2018.11.24
    cv::Mat DrawFrame();

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn    去更新最新帧的信息   lishuwei 2018.11.24
    cv::Mat mIm;                        //关键帧矩阵       lishuwei 2018.11.24
    int N;                              //关键帧序列号      lishuwei 2018.11.24
    vector<cv::KeyPoint> mvCurrentKeys; //关键帧的关键点向量 lishuwei 2018.11.24
    vector<bool> mvbMap, mvbVO;         //
    bool mbOnlyTracking;                //ORBSLAM2的运行模式，如果为true，则只起到定位模式，不会产生稀疏点云地图，若flase，则为完全的slam功能 lishuwei 2018.11.24
    int mnTracked, mnTrackedVO;         //mnTracked 正常跟踪； mnTrackedVO为只有只有定位
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;

    Map* mpMap;

    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
