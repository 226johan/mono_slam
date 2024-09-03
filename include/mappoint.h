//
// Created by johan on 2024/9/3.
//

#ifndef MONO_SLAM_MAPPOINT_H
#define MONO_SLAM_MAPPOINT_H

#include"./common_include.h"

namespace mono_slam {
    struct Feature;
    struct Frame;

    /**
     * @brief 路标点类
     * 特征点在三角化后形成路标点
     * @author johan
     */
    class MapPoint{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            typedef std::shared_ptr<MapPoint> Ptr;
            unsigned long id_ = 0; // id
            bool is_outlier_ = false; // 是否是异常点
            Vec3 pos_ = Vec3::Zero(); // position in world
            std::mutex data_mutex_;
            int observed_times_ = 0; // being observed times   observed 匹配
            std::list<std::weak_ptr<Feature>> observations_; // 观测到的特征

            MapPoint(){}

            MapPoint(long id, Vec3 position);

            /**
             * @brief 获取路标点的位置
             */
            Vec3 Pos(){
                std::unique_lock<std::mutex> lock(data_mutex_);
                return pos_;
            }

            /**
             * @brief 设置路标点的位置
             */
            void SetPos(const Vec3& pos){
                std::unique_lock<std::mutex> lock(data_mutex_);
                pos_ = pos;
            }

            /**
             * @brief 添加一个新的观测到路标点
             */
            void AddObservation(const std::shared_ptr<Feature>& feature)
            {
                std::unique_lock<std::mutex> lck(data_mutex_);
                observations_.push_back(feature);
                observed_times_++;
            }

            /**
             * @brief 移除一个观测
             */
            void RemoveObservation(std::shared_ptr<Feature> feat);

            /**
             * @brief 获取路标点的所有观测
             */
            std::list<std::weak_ptr<Feature>> GetObs(){
                std::unique_lock<std::mutex> lck(data_mutex_);
                return observations_;
            }

            // 工厂构建模式 创建新的路标点
            static MapPoint::Ptr CreateNewMappoint();

    };
}   // namespace mono_slam
#endif //MONO_SLAM_MAPPOINT_H
