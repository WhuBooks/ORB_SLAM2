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


#ifndef TRACKING_H
#define TRACKING_H

#include <unistd.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{
	
	class Viewer;
	
	class FrameDrawer;
	
	class Map;
	
	class LocalMapping;
	
	class LoopClosing;
	
	class System;
	
	class Tracking
	{
	
	public:
		Tracking (System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap,
				  KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor);
		
		// Preprocess the input and call Track(). Extract features and performs stereo matching.
		cv::Mat GrabImageStereo (const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp);
		
		cv::Mat GrabImageRGBD (const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp);
		
		cv::Mat GrabImageMonocular (const cv::Mat &im, const double &timestamp);
		
		void SetLocalMapper (LocalMapping *pLocalMapper);
		
		void SetLoopClosing (LoopClosing *pLoopClosing);
		
		void SetViewer (Viewer *pViewer);
		
		// Load new settings
		// The focal lenght should be similar or scale prediction will fail when projecting points
		// TODO: Modify MapPoint::PredictScale to take into account focal lenght
		void ChangeCalibration (const string &strSettingPath);
		
		// Use this function if you have deactivated local mapping and you only want to localize the camera.
		void InformOnlyTracking (const bool &flag);
	
	
	public:
		
		// Tracking states
		enum eTrackingState
		{
			SYSTEM_NOT_READY = -1,
			NO_IMAGES_YET = 0,
			NOT_INITIALIZED = 1,
			OK = 2,
			LOST = 3
		};
		
		eTrackingState mState;
		eTrackingState mLastProcessedState;
		
		// Input sensor
		int mSensor;
		
		// Current Frame
		Frame mCurrentFrame;
		cv::Mat mImGray;
		
		// Initialization Variables (Monocular)
		std::vector<int> mvIniLastMatches;
		std::vector<int> mvIniMatches;
		std::vector<cv::Point2f> mvbPrevMatched;
		std::vector<cv::Point3f> mvIniP3D;
		Frame mInitialFrame;
		
		// Lists used to recover the full camera trajectory at the end of the execution.
		// Basically we store the reference keyframe for each frame and its relative transformation
		list<cv::Mat> mlRelativeFramePoses;
		list<KeyFrame *> mlpReferences;
		list<double> mlFrameTimes;
		list<bool> mlbLost;
		
		// True if local mapping is deactivated and we are performing only localization
		bool mbOnlyTracking;
		
		void Reset ();
	
	protected:
		
		// Main tracking function. It is independent of the input sensor.
		/// 分为三种模式:初始化,建图,定位
		/// 初始化: 分为单目模式和双目/深度两种模式,对应了有深度信息和无深度信息两种情况
		/// 建图: 分为速度模型跟踪,关键帧匹配跟踪和关键帧数据库匹配跟踪,然后使用motion ba优化当前帧位姿
		/// 定位: 跟踪方式与建图一致,额外增加判断当前定位结果是否是odometry模式,odometry模式下不再进行motion ba
		void Track ();
		
		// Map initialization for stereo and RGB-D
		void StereoInitialization ();
		
		// Map initialization for monocular
		/// 如果在两帧内能够匹配到足够多的orb特征，尝试初始化地图
		void MonocularInitialization ();
		
		/// 使用单目初始化成功的两个关键帧创建地图
		void CreateInitialMapMonocular ();
		
		void CheckReplacedInLastFrame ();
		
		/// 使用词袋模型对当前帧和历史关键帧进行匹配
		/// 由匹配结果得到当前帧与地图点之间的关联
		/// 使用PnP算法得到当前帧的位姿
		bool TrackReferenceKeyFrame ();
		
		void UpdateLastFrame ();
		
		/// 根据匀速模型计算当前帧的位姿
		/// 匹配当前帧和最后一帧之间的特征点,判断当前计算结果的准确性
		bool TrackWithMotionModel ();
		
		/// 在关键帧数据库中寻找所有与当前帧特征匹配数量足够的关键帧
		/// 在Ransac框架下使用这些关键帧和当前帧做PnP算法获取最优位姿
		bool Relocalization ();
		
		/// 更新局部地图的关键帧和地图点
		void UpdateLocalMap ();
		
		/// 在局部关键帧集合中寻找所有的地图点
		void UpdateLocalPoints ();
		
		/// 查找与当前帧共享视野的所有关键帧
		/// 查找与这些关键帧有着通视关系的关键帧
		void UpdateLocalKeyFrames ();
		
		/// 更新局部地图
		/// 使用更新后的局部地图优化当前帧位姿
		bool TrackLocalMap ();
		
		/// 在局部地图点中再一次寻找那些点被当前帧跟踪到了
		void SearchLocalPoints ();
		
		bool NeedNewKeyFrame ();
		
		void CreateNewKeyFrame ();
		
		// In case of performing only localization, this flag is true when there are no matches to
		// points in the map. Still tracking will continue if there are enough matches with temporal points.
		// In that case we are doing visual odometry. The system will try to do relocalization to recover
		// "zero-drift" localization to the map.
		bool mbVO;
		
		//Other Thread Pointers
		LocalMapping *mpLocalMapper;
		LoopClosing *mpLoopClosing;
		
		//ORB
		ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
		ORBextractor *mpIniORBextractor;
		
		//BoW
		ORBVocabulary *mpORBVocabulary;
		KeyFrameDatabase *mpKeyFrameDB;
		
		// Initalization (only for monocular)
		Initializer *mpInitializer;
		
		//Local Map
		KeyFrame *mpReferenceKF;
		std::vector<KeyFrame *> mvpLocalKeyFrames;
		std::vector<MapPoint *> mvpLocalMapPoints;
		
		// System
		System *mpSystem;
		
		//Drawers
		Viewer *mpViewer;
		FrameDrawer *mpFrameDrawer;
		MapDrawer *mpMapDrawer;
		
		//Map
		Map *mpMap;
		
		//Calibration matrix
		cv::Mat mK;
		cv::Mat mDistCoef;
		float mbf;
		
		//New KeyFrame rules (according to fps)
		int mMinFrames;
		int mMaxFrames;
		
		// Threshold close/far points
		// Points seen as close by the stereo/RGBD sensor are considered reliable
		// and inserted from just one frame. Far points requiere a match in two keyframes.
		float mThDepth;
		
		// For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
		float mDepthMapFactor;
		
		//Current matches in frame
		int mnMatchesInliers;
		
		//Last Frame, KeyFrame and Relocalisation Info
		KeyFrame *mpLastKeyFrame;
		Frame mLastFrame;
		unsigned int mnLastKeyFrameId;
		unsigned int mnLastRelocFrameId;
		
		//Motion Model
		cv::Mat mVelocity;
		
		//Color order (true RGB, false BGR, ignored if grayscale)
		bool mbRGB;
		
		list<MapPoint *> mlpTemporalPoints;
	};
	
} //namespace ORB_SLAM

#endif // TRACKING_H
