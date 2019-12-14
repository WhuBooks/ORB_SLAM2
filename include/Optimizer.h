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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{
	
	class LoopClosing;
	
	class Optimizer
	{
	public:
		/// 对指定的关键帧和地图点进行ba
		void static BundleAdjustment (const std::vector<KeyFrame *> &vpKF, const std::vector<MapPoint *> &vpMP,
									  int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
									  const bool bRobust = true);
		
		/// 对地图中所有的关键帧和地图点进行ba
		void static GlobalBundleAdjustemnt (Map *pMap, int nIterations = 5, bool *pbStopFlag = NULL,
											const unsigned long nLoopKF = 0, const bool bRobust = true);
		
		/// 对当前关键帧及其相邻关键帧和地图点进行structure ba
		void static LocalBundleAdjustment (KeyFrame *pKF, bool *pbStopFlag, Map *pMap);
		
		/// 对指定帧进行四次motion ba,每次motion ba的结果会作为下一次的初始状态
		int static PoseOptimization (Frame *pFrame);
		
		// if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
		/// 基于回环检测结果的全局ba
		/// 首先将当前回环检测结果加入图中
		/// 对于每个关键帧,存在以下几种边:
		/// 生产树上的父节点,回环边,通视图中的局部回环边
		void static OptimizeEssentialGraph (Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
											const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
											const LoopClosing::KeyFrameAndPose &CorrectedSim3,
											const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
											const bool &bFixScale);
		
		// if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
		/// 根据两个关键帧之间ransac的sim3结果连续进行两次优化
		static int OptimizeSim3 (KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches1,
								 g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
	};
	
} //namespace ORB_SLAM

#endif // OPTIMIZER_H
