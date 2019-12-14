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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{
	
	class Tracking;
	
	class LocalMapping;
	
	class KeyFrameDatabase;
	
	
	class LoopClosing
	{
	public:
		
		typedef pair<set<KeyFrame *>, int> ConsistentGroup;
		typedef map<KeyFrame *, g2o::Sim3, std::less<KeyFrame *>,
					Eigen::aligned_allocator<std::pair<const KeyFrame *, g2o::Sim3> > > KeyFrameAndPose;
	
	public:
		
		LoopClosing (Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale);
		
		void SetTracker (Tracking *pTracker);
		
		void SetLocalMapper (LocalMapping *pLocalMapper);
		
		// Main function
		/// 检测连续回环的出现
		/// 在连续回环中初始化当前关键帧的sim3变换
		/// 将当前关键帧与回环关键帧及其相关的关键帧一起放入ba中优化
		void Run ();
		
		void InsertKeyFrame (KeyFrame *pKF);
		
		void RequestReset ();
		
		// This function will run in a separate thread
		void RunGlobalBundleAdjustment (unsigned long nLoopKF);
		
		bool isRunningGBA ()
		{
			unique_lock<std::mutex> lock(mMutexGBA);
			return mbRunningGBA;
		}
		
		bool isFinishedGBA ()
		{
			unique_lock<std::mutex> lock(mMutexGBA);
			return mbFinishedGBA;
		}
		
		void RequestFinish ();
		
		bool isFinished ();
		
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	protected:
		
		bool CheckNewKeyFrames ();
		
		/// 根据词袋模型计算当前帧与其临近帧的相似度,并以此为阈值寻找历史中的相似帧
		/// 只有在连续检测到回环的出现后,才会开启回环校正
		bool DetectLoop ();
		
		/// 对于检测到的连续回环,建立多个sim3解算方法,在ransac框架下获取最优的计算结果
		bool ComputeSim3 ();
		
		void SearchAndFuse (const KeyFrameAndPose &CorrectedPosesMap);
		
		/// 首先将当前关键帧附近的关键帧以及相关地图点转换为sim3模式
		/// 然后以初始位姿为准搜索回环帧附近关键帧及其相关地图点
		/// 将当前关键帧和回环帧及其附近帧一起放入ba中进行优化
		void CorrectLoop ();
		
		void ResetIfRequested ();
		
		bool mbResetRequested;
		std::mutex mMutexReset;
		
		bool CheckFinish ();
		
		void SetFinish ();
		
		bool mbFinishRequested;
		bool mbFinished;
		std::mutex mMutexFinish;
		
		Map *mpMap;
		Tracking *mpTracker;
		
		KeyFrameDatabase *mpKeyFrameDB;
		ORBVocabulary *mpORBVocabulary;
		
		LocalMapping *mpLocalMapper;
		
		std::list<KeyFrame *> mlpLoopKeyFrameQueue;
		
		std::mutex mMutexLoopQueue;
		
		// Loop detector parameters
		float mnCovisibilityConsistencyTh;
		
		// Loop detector variables
		KeyFrame *mpCurrentKF;
		KeyFrame *mpMatchedKF;
		std::vector<ConsistentGroup> mvConsistentGroups;
		std::vector<KeyFrame *> mvpEnoughConsistentCandidates;
		std::vector<KeyFrame *> mvpCurrentConnectedKFs;
		std::vector<MapPoint *> mvpCurrentMatchedPoints;
		std::vector<MapPoint *> mvpLoopMapPoints;
		cv::Mat mScw;
		g2o::Sim3 mg2oScw;
		
		long unsigned int mLastLoopKFid;
		
		// Variables related to Global Bundle Adjustment
		bool mbRunningGBA;
		bool mbFinishedGBA;
		bool mbStopGBA;
		std::mutex mMutexGBA;
		std::thread *mpThreadGBA;
		
		// Fix scale in the stereo/RGB-D case
		bool mbFixScale;
		
		
		bool mnFullBAIdx;
	};
	
} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
