//
//  projection_facor.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef projection_facor_hpp
#define projection_facor_hpp

#include <stdio.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utility.hpp"

/**
 * 视觉约束
 * 2: 殘差長度(err_x, err_y)
 * 7: 第1個優化參數pose_i的長度(para_Pose[imu_i]=(px,py,pz,qx,qy,qz,qw) )
 * 7: 第2個優化參數pose_j的長度(para_Pose[imu_j])
 * 7: 第3個優化參數外參的長度(para_Ex_Pose[0])
 * 1: 第4個優化參數feature_inverse_depth的長度(para_Feature[feature_index])
 */
class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>
{
public:
    ProjectionFactor(const Eigen::Vector3d &_pts_i,
                     const Eigen::Vector3d &_pts_j);
    
    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const;
    
public:
    /**
     * 对应8.4.1节中的图8-2 因为在之前的程序中计算特征点位置的时候保存的是其在归一化平面中的特征点，
     * 所以对特征点深度的定义就变成了实际3D点的Z坐标，整个代码都是将Z轴坐标作为深度的。
     */
    Eigen::Vector3d pts_i;  // 帧坐标系下，是否已投影到单位球？
    Eigen::Vector3d pts_j;  // 帧坐标系下，是否已投影到单位球？
    
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

#endif /* projection_facor_hpp */
