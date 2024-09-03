//
// Created by johan on 2024/9/3.
//

#ifndef MONO_SLAM_FEATURE_H
#define MONO_SLAM_FEATURE_H

#include<opencv2/features2d.hpp>
#include<memory>
#include"./common_include.h"

namespace mono_slam {
    struct Frame;
    struct MapPoint;

    class Feature {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Feature> Ptr;

            std::weak_ptr<Frame> frame_; // 持有该feature的frame
            cv::KeyPoint position_; // 2d特征点位置
            std::weak_ptr<MapPoint> map_point_; // 与该特征点关联的地图点

            bool is_outlier_ = false; // 是否是异常点
            bool is_on_left_image_ = true; // 是否在左图中,false表示在右图中

        public:
            Feature(){}

            Feature(std::shared_ptr<Frame> frame,const cv::KeyPoint& kp)
                : frame_(frame), position_(kp) {}
    };
}

#endif //MONO_SLAM_FEATURE_H
