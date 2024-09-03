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

    /**
     * @brief 2d 特征点类
     * 在三角化后会被关联一个地图点
     * @note 该类不应被直接实例化，应使用Ptr或shared_ptr
     * @author johan
     */
    class Feature {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Feature> Ptr;

            // frame和mappoint实际所有权归地图所有,所以使用weak_ptr避免shared_ptr产生的循环引用
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
}   // namespace mono_slam

#endif //MONO_SLAM_FEATURE_H
