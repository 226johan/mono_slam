//
// Created by johan on 2024/9/4.
//

#ifndef MONO_SLAM_ALGORITHM_H
#define MONO_SLAM_ALGORITHM_H

#include"./common_include.h"

namespace mono_slam {
    /**
     * @brief 三角化算法
     * @param poses   相机位姿
     * @param points   特征点在相机坐标系下的坐标
     * @param pt_world 三角测量后得到的世界坐标
     * @return  三角测量是否成功
     * @note 使用inline修饰(内联函数),提升运行效率
     */
    inline bool triangulation(const std::vector<SE3>& poses, const std::vector<Vec3> points, Vec3& pt_world){
        // 初始化
        MatXX A(2*poses.size(), 4);
        VecX b(2*poses.size());
        b.setZero();
        // 填充矩阵A
        for(size_t i=0; i<poses.size(); ++i) {
            // 将相机位姿转换成3x4矩阵
            Mat34 m = poses[i].matrix3x4();
            // 填充A矩阵
            A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
            A.block<1, 4>(2 * i + 1, 0) = points[i][i] * m.row(2) - m.row(1);
        }
        // 分解奇异值
        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        // 计算世界坐标   *需要先归一化
        pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();
        // 检查解的质量
        if(svd.singularValues()[3] / svd.singularValues()[0] < 1e-2)
        {
            // 解的质量不好,放弃
            return true;
        }
        return false;
    }

    // converters
    /**
     * @brief 转换cv::Point2f到Vec2
     * @param p   cv::Point2f
     * @return    Vec2
     */
    inline Vec2 toVec2(const cv::Point2f p){return Vec2(p.x, p.y);}

}    // namespace mono_slam
#endif //MONO_SLAM_ALGORITHM_H
