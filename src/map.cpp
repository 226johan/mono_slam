//
// Created by johan on 2024/9/3.
//

#include"../include/map.h"
#include"../include/feature.h"

namespace mono_slam {
    void Map::InsertKeyframe(Frame::Ptr frame) {
        // 更新当前帧 表示当前处理的关键帧
        current_frame_ = frame;
        // 检查关键帧是否已经存在
        if(keyframes_.find(frame->keyframe_id_) == keyframes_.end()){
            // 如果不存在，则插入 键是关键帧的id,值是关键帧本身
            keyframes_.insert(make_pair(frame->keyframe_id_, frame));
            active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        } else {
            // 如果存在，则更新
            keyframes_[frame->keyframe_id_]=frame;
            active_keyframes_[frame->keyframe_id_]=frame;
        }
        // 检查活动的关键帧数量,超过则删除旧的
        if(active_keyframes_.size() > num_active_keyframes_){
            RemoveOldKeyFrame();
        }
    }

    void Map::InsertMapPoint(MapPoint::Ptr map_point) {
        // 检查地图点是否已经存在
        if(landmarks_.find(map_point->id_) == landmarks_.end()){
            // 如果不存在，则插入 键是地图点的id,值是地图点本身
            landmarks_.insert(make_pair(map_point->id_, map_point));
            active_landmarks_.insert(make_pair(map_point->id_, map_point));
        } else {
            // 如果存在，则更新
            landmarks_[map_point->id_]=map_point;
            active_landmarks_[map_point->id_]=map_point;
        }

    }

    void Map::RemoveOldKeyFrame() {
        if(current_frame_ == nullptr) return;
        // 寻找与当前帧距离最近和最远的两个关键帧
        double max_dis = 0,min_dis = 9999;
        double max_kf_id = 0,min_kf_id = 0;
        auto Twc = current_frame_->Pose().inverse();
        for(auto& kf : keyframes_){
            // 如果是当前帧，则跳过
            if(kf.second == current_frame_) continue;
            // 计算当前遍历的关键帧与当前帧的距离
            auto dis = (kf.second->Pose() * Twc).log().norm();
            // 更新距离
            if(dis > max_dis){
                max_dis = dis;
                max_kf_id = kf.first;
            }
            if(dis < min_dis){
                min_dis = dis;
                min_kf_id = kf.first;
            }
        }

        const double min_dis_th = 0.2;  // 距离阈值
        Frame::Ptr frame_to_remove = nullptr;
        if(max_dis < min_dis_th){
            // 如果存在很近的关键帧，则删除距离最小的关键帧
            frame_to_remove = keyframes_.at(min_kf_id);
        } else {
            // 否则删除距离最大的关键帧
            frame_to_remove = keyframes_.at(max_kf_id);
        }

//        LOG(INFO) << "Remove old keyframe: " << frame_to_remove->keyframe_id_;


    }
}
