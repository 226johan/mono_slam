//
// Created by johan on 2024/9/2.
//

#include "../include/camera.h"

namespace mono_slam {
    /**
     * @brief world2camera 将世界坐标转换为相机坐标
     * @param p_w  世界坐标
     * @param T_c_w  相机坐标系到世界坐标系的变换矩阵
     * @return   相机坐标
     */
    Vec3 Camera::world2camera(const Vec3& p_w,const SE3& T_c_w){
        return pose_ * T_c_w * p_w;
    }

    /**
     * @brief camera2world 将相机坐标转换为世界坐标
     * @param p_c  相机坐标
     * @param T_c_w  相机坐标系到世界坐标系的变换矩阵
     * @return   世界坐标
     */
    Vec3 Camera::camera2world(const Vec3& p_c,const SE3& T_c_w){
        return T_c_w * pose_inv_ * p_c;
    }

    /**
     * @brief camera2pixel 将相机坐标转换为像素坐标
     * @param p_c  相机坐标
     * @return  像素坐标
     */
    Vec2 Camera::camera2pixel(const Vec3& p_c){
        return Vec2(fx_ * p_c(0,0) / p_c(2,0),
                    fy_ * p_c(1,0) / p_c(2,0));
    }

    /**
     * @brief pixel2camera 将像素坐标转换为相机坐标
     * @param p_p  像素坐标
     * @param depth 深度
     * @return  相机坐标
     */
    Vec3 Camera::pixel2camera(const Vec2& p_p,double depth)
    {
        return Vec3((p_p(0,0) - cx_) * depth / fx_,
                    (p_p(1,0) - cy_) * depth / fy_,
                    depth);
    }

    /**
     * @brief pixel2world 将像素坐标转换为世界坐标
     * @param p_p  像素坐标
     * @param T_c_w 相机坐标系到世界坐标系的变换矩阵
     * @param depth 深度
     * @return    世界坐标
     */
    Vec3 Camera::pixel2world(const Vec2& p_p, const SE3& T_c_w, double depth){
        return camera2world(pixel2camera(p_p, depth), T_c_w);
    }

    /**
     * @brief world2pixel 将世界坐标转换为像素坐标
     * @param p_w  世界坐标
     * @param T_c_w 相机坐标系到世界坐标系的变换矩阵
     * @return
     */
    Vec2 Camera::world2pixel(const Vec3& p_w, const SE3& T_c_w)
    {
        return camera2pixel(world2camera(p_w, T_c_w));
    }
}   // namespace mono_slam