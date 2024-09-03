//
// Created by johan on 2024/9/3.
//

#ifndef MONO_SLAM_MAP_H
#define MONO_SLAM_MAP_H

#include"./common_include.h"
#include"./frame.h"
#include"./mappoint.h"

namespace mono_slam {
    /**
     * @brief 地图
     * 和地图的交互:前段调用InsertKeyframe插入新帧和地图点,后端维护地图的结构,判断outlier/剔除等
     */
    class Map {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Map> Ptr;
            typedef std::unordered_map<unsigned long , MapPoint::Ptr> LandmarksType;
            typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

            Map(){}

            // 增加关键帧
            void InsertKeyframe(Frame::Ptr frame);

            // 增加一个地图顶点
            void InsertMapPoint(MapPoint::Ptr map_point);

            // 获取所有地图点
            LandmarksType GetAllMapPoints();

            // 获取所有关键帧
            KeyframesType  GetAllKeyFrames();

            // 获取激活地图点
            LandmarksType GetActiveMapPoints();

            // 获取激活关键帧
            KeyframesType GetActiveKeyFrames();

            // 清除map中观测数量为0的点
            void CleanMap();

    private:
        // 将旧的关键点设置为不活跃状态
        void RemoveOldKeyFrame();

        std::mutex data_mutex_;
        LandmarksType landmarks_;  // 所有地图点
        LandmarksType active_landmarks_;  // 激活的地图点
        KeyframesType keyframes_;  // 所有关键帧
        KeyframesType active_keyframes_;  // 激活的关键帧

        Frame::Ptr current_frame_ = nullptr;  // 当前帧

        // settings
        int num_active_keyframes_ = 7;  // 激活的关键帧数量
    };
}     // namespace mono_slam

#endif //MONO_SLAM_MAP_H
