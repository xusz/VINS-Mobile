//
//  utility.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "utility.hpp"

Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2 {0, 0, 1.0};
    
    // 获得一个旋转矩阵把坐标ng1转换到ng2
    Eigen::Matrix3d R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = Utility::R2ypr(R0).x();
    
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
//    R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    
    // R0是从当前坐标系到世界坐标系的旋转矩阵（当前坐标系的z轴，与世界坐标系的重力g的方向对应）
    return R0;
}
