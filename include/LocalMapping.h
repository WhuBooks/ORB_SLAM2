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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{
	
	class Tracking;
	
	class LoopClosing;
	
	class Map;
	
	class LocalMapping
	{
	public:
		LocalMapping (Map *pMap, const float bMonocular);
		
		void SetLoopCloser (LoopClosing *pLoopCloser);
		
		void SetTracker (Tracking *pTracker);
		
		// Main function
		/// 处理新的关键帧
		/// 清理劣质的地图点
		/// 三角化测量新的地图点
		/// 扩大通视图范围,更新地图点信息和当前关键帧通视图信息
		/// 清理多余的关键帧
		void Run ();
		
		void InsertKeyFrame (KeyFrame *pKF);
		
		// Thread Synch
		void RequestStop ();
		
		void RequestReset ();
		
		bool Stop ();
		
		void Release ();
		
		bool isStopped ();
		
		bool stopRequested ();
		
		bool AcceptKeyFrames ();
		
		void SetAcceptKeyFrames (bool flag);
		
		bool SetNotStop (bool flag);
		
		void InterruptBA ();
		
		void RequestFinish ();
		
		bool isFinished ();
		
		int KeyframesInQueue ()
		{
			unique_lock<std::mutex> lock(mMutexNewKFs);
			return mlNewKeyFrames.size();
		}
	
	protected:
		
		bool CheckNewKeyFrames ();
		
		/// 处理新的关键帧,将地图点与关键帧关联起来
		void ProcessNewKeyFrame ();
		
		/// 计算当前关键帧与其通视关键帧之间的Fundametal矩阵
		/// 根据Fundamental矩阵寻找特征匹配点,根据视差判断匹配点的有效性
		/// 对匹配点进行线性三角化得到地图点,如果是双目相机或者深度相机,可直接取其测量结果作为地图点
		/// 根据线性三角化后的点在两个关键帧中的重投影误差判断地图点的有效性
		void CreateNewMapPoints ();
		
		/// 清理观测性不好的地图点
		void MapPointCulling ();
		
		/// 将原本的通视图搜索改为两级通视图搜索
		/// 以此更新当前关键帧的地图点和通视图
		void SearchInNeighbors ();
		
		/// 清理多余的关键帧,如果某个关键帧内的绝大多数地图点都能被至少三个关键帧观测到,则此关键帧可被认为是重复的
		void KeyFrameCulling ();
		
		/// 根据刚体变换计算两关键帧之间Fundamental矩阵
		cv::Mat ComputeF12 (KeyFrame *&pKF1, KeyFrame *&pKF2);
		
		cv::Mat SkewSymmetricMatrix (const cv::Mat &v);
		
		bool mbMonocular;
		
		void ResetIfRequested ();
		
		bool mbResetRequested;
		std::mutex mMutexReset;
		
		bool CheckFinish ();
		
		void SetFinish ();
		
		bool mbFinishRequested;
		bool mbFinished;
		std::mutex mMutexFinish;
		
		Map *mpMap;
		
		LoopClosing *mpLoopCloser;
		Tracking *mpTracker;
		
		std::list<KeyFrame *> mlNewKeyFrames;
		
		KeyFrame *mpCurrentKeyFrame;
		
		std::list<MapPoint *> mlpRecentAddedMapPoints;
		
		std::mutex mMutexNewKFs;
		
		bool mbAbortBA;
		
		bool mbStopped;
		bool mbStopRequested;
		bool mbNotStop;
		std::mutex mMutexStop;
		
		bool mbAcceptKeyFrames;
		std::mutex mMutexAccept;
	};
	
} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
