//
// Created by johan on 2024/9/3.
//

#include"../include/frame.h"

namespace mono_slam {
    Frame::Frame(long id, double time_stamp, const SE3& pose,const Mat& left, const Mat& right)
    : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

    /**
     * Create a new frame with a unique id.
     * @return Frame::Ptr
     */
    Frame::Ptr Frame::CreateFrame(){
        static long factory_id=0;
        Frame::Ptr new_frame(new Frame);
        new_frame->id_=factory_id++;
        return new_frame;
    }

    /**
     * Set the current frame as a keyframe.
     */
    void Frame::SetKeyFrame() {
        static long keyframe_factory_id=0;
        is_keyframe_=true;
        keyframe_id_=keyframe_factory_id++;
    }
}   // namespace mono_slam
