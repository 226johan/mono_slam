//
// Created by johan on 2024/9/3.
//

#include"../include/mappoint.h"
#include"../include/feature.h"


namespace mono_slam {
    MapPoint::MapPoint(long id, Vec3 position) : id_(id), pos_(position){}

    MapPoint::Ptr MapPoint::CreateNewMappoint() {
        static long factory_id = 0;
        MapPoint::Ptr new_mappoint(new MapPoint);
        new_mappoint->id_ = factory_id++;
        return new_mappoint;
    }

    void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat) {
        // feature 可能会被判断成outlier，因此observations_发生改动时，需要加锁
        std::unique_lock<std::mutex> lock(data_mutex_);
        for(auto iter = observations_.begin(); iter != observations_.end(); iter++){
            // 产生了类型转换
            // 尝试将 std::weak_ptr<Feature> 转化为 std::shared_ptr<Feature>
            if(iter->lock() == feat) {
                // 找到了要删除的观测，删除它
                observations_.erase(iter);
                // 观测的特征点指针置空,表示观测与地图点的关系解除
                feat->map_point_.reset();
                // 地图点的观测次数减一
                observed_times_--;
                break;
            }
        }
    }



}   // namespace mono_slam
