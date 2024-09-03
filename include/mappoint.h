//
// Created by johan on 2024/9/3.
//

#ifndef MONO_SLAM_MAPPOINT_H
#define MONO_SLAM_MAPPOINT_H

#include"./common_include.h"

namespace mono_slam {
    struct Feature;
    struct Frame;

    class MapPoint{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            typedef std::shared_ptr<MapPoint> Ptr;
            unsigned long id_ = 0; // id
            bool is_outlier_ = false;
            Vec3 pos_ = Vec3::Zero(); // position in world
            std::mutex data_mutex_;
            int observed_times_ = 0; // being observed times   observed 匹配
            std::list<std::weak_ptr<Feature>> observations_; // 观测到的特征
    };
}
#endif //MONO_SLAM_MAPPOINT_H
