//
// Created by johan on 2024/9/4.
//

#ifndef MONO_SLAM_FRONTEND_H
#define MONO_SLAM_FRONTEND_H

#include <opencv2/features2d.hpp>
#include"./common_include.h"
#include"./frame.h"
#include "./map.h"

namespace mono_slam {

    class Backend;
    class Viewer;

    enum class FrontendStatus {INITING, TRACKING_GOOD, TRACKING_BAD, LOST};

    /**
     * 前端
     * 估计当前帧pose,在满足关键帧条件时向地图中加入关键帧并触发优化
     */

    class Frontend {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frontend> Ptr;

        Frontend();

        // 添加一个帧并计算其定位结果
        bool AddFrame(Frame::Ptr frame);

        // set
        void SetMap(Map::Ptr map){map_ = map;}

        void SetBackend(std::shared_ptr<Backend> backend){backend_ = backend;}

        void SetViewer(std::shared_ptr<Viewer> viewer){viewer_ = viewer;}

        FrontendStatus GetStatus() const {return status_;}

        void SetCameras(Camera::Ptr left, Camera::Ptr right){
            camera_left_ = left;
            camera_right_ = right;
        }

    private:

        bool Track();

        bool Reset();

        int TrackLastFrame();

        int EstimateCurrentPose();

        bool InsertKeyFrame();

        bool StereoInit();

        int DetectFeatures();

        int FindFeatures();

        int FindFeaturesInRight();

        bool BuildInitMap();

        int TriangulateNewPoints();

        void SetObservationsForKeyFrame();

        // data
        FrontendStatus status_ = FrontendStatus::INITING;

        Frame::Ptr current_frame_ = nullptr;  // 当前帧
        Frame::Ptr last_frame_ = nullptr;  // 上一帧
        Camera::Ptr camera_left_ = nullptr;  // 左相机
        Camera::Ptr camera_right_ = nullptr;  // 右相机

        Map::Ptr map_ = nullptr;  // 地图
        std::shared_ptr<Backend> backend_ = nullptr;  // 后端
        std::shared_ptr<Viewer> viewer_ = nullptr;  // 可视化

        SE3 relative_pose_ = SE3();  // 当前帧与上一帧的相对运动,用于估计当前帧pose初值

        int tracking_inliers_ = 0;

        // params
        int num_features_ = 200;  // 特征点数目
        int num_features_init_ = 100;  // 初始特征点数目
        int num_features_tracking_ = 50;
        int num_features_tracking_bad_ = 20;
        int num_features_needed_for_keyframes_ = 80;

        // utilities
        cv::Ptr<cv::GFTTDetector> gtff_;  // 特征点检测器
    };

}   // namespace mono_slam

#endif //MONO_SLAM_FRONTEND_H
