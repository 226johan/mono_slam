//
// Created by johan on 2024/9/2.
//

#ifndef MONO_SLAM_CAMERA_H
#define MONO_SLAM_CAMERA_H

#include "common_include.h"
namespace mono_slam {
    /*
     * 相机类
     * @author johan
     */
    class Camera {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;     // 确保eigen动态分配的内存对齐\
        typedef std::shared_ptr<Camera> Ptr;

        double fx_=0, fy_=0, cx_=0, cy_=0, baseline_=0; // 相机内参
        SE3 pose_; // 相机位姿
        SE3 pose_inv_; // 相机逆位姿

        Camera()=default;
        Camera(double fx, double fy, double cx, double cy, double baseline, const SE3& pose)
            : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {pose_inv_ = pose.inverse();}

        SE3 pose() const {return pose_;}

        // 相机内参矩阵
        Mat33 K() const {
            Mat33 K;
            K << fx_, 0, cx_,
                 0, fy_, cy_,
                 0, 0, 1;
            return K;
        }

        // 世界坐标,相机坐标,像素坐标的转换
        Vec3 world2camera(const Vec3& p_w,const SE3& T_c_w);

        Vec3 camera2world(const Vec3& p_c,const SE3& T_c_w);

        Vec2 camera2pixel(const Vec3& p_c);

        Vec3 pixel2camera(const Vec2& p_p,double depth = 1.0);

        Vec3 pixel2world(const Vec2& p_p, const SE3& T_c_w, double depth = 1.0);

        Vec2 world2pixel(const Vec3& p_w, const SE3& T_c_w);

    };

} // namespace mono_slam
#endif //MONO_SLAM_CAMERA_H
