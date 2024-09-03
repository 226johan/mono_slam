//
// Created by johan on 2024/9/2.
//

#ifndef MONO_SLAM_FRAME_H
#define MONO_SLAM_FRAME_H

#include"common_include.h"
#include"camera.h"

namespace mono_slam {

    struct MapPoint;
    struct Feature;

    /**
     * 帧
     * 每一帧分配独立id,关键帧分配关键帧id
     */
    struct Frame {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Frame> Ptr;

            unsigned long id_ = 0; // 当前帧id
            unsigned long keyframe_id_ = 0; // 关键帧id
            bool is_keyframe_ = false; // 是否为关键帧
            double time_stamp_ ; // 时间戳
            SE3 pose_; // T_w_c
            std::mutex pose_mutex_; // pose_的数据锁
            cv::Mat left_img_, right_img_; // 左右图像

            // 特征点
            // feature of left
            std::vector<std::shared_ptr<Feature>> features_left_;
            // feature of right , set to empty if no corresponding
            std::vector<std::shared_ptr<Feature>> features_right_;

        public:  // data members
            Frame(){}
            Frame(long id, double time_stamp, const SE3& pose,const Mat& left, const Mat& right);

            // set and get pose ,thread safe
            SE3 Pose(){
                // *使用智能锁保护pose_的数据访问,保证线程安全
                std::unique_lock<std::mutex> lck(pose_mutex_);
                return pose_;
            }

            void SetPose(const SE3& pose){
                std::unique_lock<std::mutex> lck(pose_mutex_);
                pose_ = pose;
            }

            // 设置为关键帧并分配关键帧id
            void SetKeyFrame();

            // 工厂构建模式,分配id
            static std::shared_ptr<Frame> CreateFrame();
            }


    };


} // namespace mono_slam





#endif //MONO_SLAM_FRAME_H
