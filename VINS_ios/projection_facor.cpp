//
//  projection_facor.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "projection_facor.hpp" 

Eigen::Matrix2d ProjectionFactor::sqrt_info;
double ProjectionFactor::sum_t;

ProjectionFactor::ProjectionFactor(const Eigen::Vector3d &_pts_i,
                                   const Eigen::Vector3d &_pts_j)
                                : pts_i(_pts_i),
                                  pts_j(_pts_j)
{
};

/**
 * 视觉重投影的残差约束
 * 重投影过程:i帧中图像中的点=>i帧相机系=>i帧body系=>world系=>j帧body系=>j帧相机系
 */
bool ProjectionFactor::Evaluate(double const *const *parameters,
                                double *residuals,
                                double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0],
                       parameters[0][1],
                       parameters[0][2]);
    
    Eigen::Quaterniond Qi(parameters[0][6],
                          parameters[0][3],
                          parameters[0][4],
                          parameters[0][5]);
    
    Eigen::Vector3d Pj(parameters[1][0],
                       parameters[1][1],
                       parameters[1][2]);
    
    Eigen::Quaterniond Qj(parameters[1][6],
                          parameters[1][3],
                          parameters[1][4],
                          parameters[1][5]);
    
    Eigen::Vector3d tic(parameters[2][0],
                        parameters[2][1],
                        parameters[2][2]);
    
    Eigen::Quaterniond qic(parameters[2][6],
                           parameters[2][3],
                           parameters[2][4],
                           parameters[2][5]);
    
    //！将第i帧下的3D点转到第j帧坐标系下
    //！转到归一化平面，得到归一化平面上点P
    
    /**
     * 重投影过程:i帧中图像中的点=>i帧相机系=>i帧body系=>world系=>j帧body系=>j帧相机系
     * 对应8.4.1节的公式（8-24）
     */
    
    // i时刻相机坐标系下的map point的逆深度
    double inv_dep_i = parameters[3][0];
    // i时刻相机坐标系下的map point坐标
    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;        // pt in ith camera frame
    // i时刻IMU坐标系下的map point坐标
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;    // pt in ith body frame
    // 世界坐标系下的map point坐标
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;             // pt in world frame
    // 在j时刻imu坐标系下的map point坐标
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj); // pt in jth body frame
    // 在j时刻相机坐标系下的map point坐标
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic); // pt in jth camera frame
    
    /**
     * 对应8.4.1节中的图8-2 因为在之前的程序中计算特征点位置的时候保存的是其在归一化平面中的特征点，
     * 所以对特征点深度的定义就变成了实际3D点的Z坐标，整个代码都是将Z轴坐标作为深度的。
     */
    double dep_j = pts_camera_j.z();
    // 残差，对应8.4.1节的公式（8-22）
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
    residual = sqrt_info * residual;
    
//    //！求取切平面上的误差
//#ifdef UNIT_SPHERE_ERROR
//    residual =  tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
//    //！求取普通的误差
//#else
//    double dep_j = pts_camera_j.z();
//    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
//#endif

    if (jacobians)
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);
        
        reduce << 1./dep_j,  0,         -pts_camera_j(0)/(dep_j*dep_j),
                  0,         1./dep_j,  -pts_camera_j(1)/(dep_j*dep_j);
        
        reduce = sqrt_info * reduce;
        
        // 对应8.4.3节中的公式（8-38）
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            
            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);
            
            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }
        
        // 对应8.4.3节中的公式（8-39）
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
            
            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric.transpose() * Utility::skewSymmetric(pts_imu_j);
            
            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
        
        // 对应8.4.3节中的公式（8-40）
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            
            jaco_ex.leftCols<3>() = ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
            
            Eigen::Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
            jaco_ex.rightCols<3>() = -tmp_r * Utility::skewSymmetric(pts_camera_i) + Utility::skewSymmetric(tmp_r * pts_camera_i) + Utility::skewSymmetric(ric.transpose() * (Rj.transpose() * (Ri * tic + Pi - Pj) - tic));
           
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        
        // 对应8.4.3节中的公式（8-41）
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
#if 1
            jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
#else
            jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i;
#endif
        }
    }
    
    return true;
}

