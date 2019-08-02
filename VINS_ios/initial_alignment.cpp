//
//  initial_aligment.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/12/26.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "initial_alignment.hpp"

/**
 * 根据视觉SFM的结果来估测陀螺仪的Bias，对应论文V-B-1
 * 注意：得到了新的Bias后，对应的预积分需要repropagate修正陀螺仪的bias
 *
 * 在滑动窗口中，每两帧之间的相对旋转与IMU预积分产生的旋转进行最小二乘法优化，用ldlt求解
 */
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame,
                        Vector3d* Bgs)
{
    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero();
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    
    // 用的是 all frames
    // 对应7.3.1节
    for (frame_i = all_image_frame.begin();
         next(frame_i) != all_image_frame.end();
         frame_i++)
    {
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();
        // 視覺給出的i幀->j幀的旋轉(四元數)
        // frame_i->second.R.transpose() == frame_i->second.R.inverse() ???
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
        
        // IMU預積分中旋轉對gyro bias的jacobian
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    
    // 当求解方程组的系数矩阵是对称矩阵时，用LDLT分解法可以简化程序设计并减少计算量
    // A矩阵一定是一个正定矩阵（实对称矩阵）
    // 对应7.3.1节中的公式（7-13）
    delta_bg = A.ldlt().solve(b);
    cout << " delta_bg ! " << delta_bg.transpose() << endl;
    
    // delta_bg 每一帧都会累积误差
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        Bgs[i] += delta_bg;
    }
    
    // 求解出陀螺仪的 bias 后，需要对 IMU 预积分值进行重新计算
    for (frame_i = all_image_frame.begin();
         next(frame_i) != all_image_frame.end();
         frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
    }
}

/**
 * Algorithm 1 to find b1 b2
 * 在半径为G的半球找到切面的一对正交基
 * 求法跟论文不太一致，但是没影响
 */
MatrixXd TangentBasis(Vector3d &g0)
{
    Vector3d b, c;
    Vector3d a = g0.normalized();
    Vector3d tmp(0, 0, 1);
    
    if (a == tmp)
    {
        tmp << 1, 0, 0;
    }
    
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    
    MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    
    return bc;
}

/**
 * 7.3.3节，进一步细化重力加速度，提高估计值的精度
 *
 * see V-B-3 in Paper
 *  1. 按照论文思路，重力向量是由重力大小所约束的，论文中使用半球加上半球切面来参数化重力
 *  2. 然后迭代求得w1,w2
 */
void RefineGravity(map<double, ImageFrame> &all_image_frame,
                   Vector3d &g, VectorXd &x)
{
    Vector3d g0 = g.normalized() * G_NORM;
    Vector3d lx, ly;
    
    int all_frame_count = (int)all_image_frame.size();
    int n_state = all_frame_count * 3 + 2 + 1;
    
    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();
    
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    
    for (int k = 0; k < 4; k++)
    {
        MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0);
        int i = 0;
        
        for (frame_i = all_image_frame.begin();
             next(frame_i) != all_image_frame.end();
             frame_i++, i++)
        {
            frame_j = next(frame_i);
            
            MatrixXd tmp_A(6, 9);
            tmp_A.setZero();
            VectorXd tmp_b(6);
            tmp_b.setZero();
            
            double dt = frame_j->second.pre_integration->sum_dt;
            
            /**
             * tmp_A 对应公式（7-30）的 H
             * x 对应公式（7-30）
             * tmp_b 对应公式（7-29）
             */
            // H_00
            tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
            // H_02
            tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
            // H_03
            tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;
            
            Vector3d TIC; // Translate of camera to imu
            TIC << TIC_X, TIC_Y, TIC_Z;
            
            // b_00
            tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC - TIC - frame_i->second.R.transpose() * dt * dt / 2 * g0;
            //tmp_b.block<3, 1>(0, 0) = imu_factors[i + 1]->pre_integration.delta_p;
            
            // H_10
            tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
            // H_11
            tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
            // H_12
            tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity() * lxly;
            
            // b_10
            tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * Matrix3d::Identity() * g0;
            
            Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
            //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
            //MatrixXd cov_inv = cov.inverse();
            cov_inv.setIdentity();
            
            // 对应公式（7-31）
            // H^T*H
            MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
            // H^T*b
            VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;
            
            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();
            
            A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
            b.tail<3>() += r_b.tail<3>();
            
            A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
            A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
        }
        
        // 对应公式（7-31），当求解方程组的系数矩阵是对称矩阵时，用LDLT分解法可以简化程序设计并减少计算量
        A = A * 1000.0;
        b = b * 1000.0;
        x = A.ldlt().solve(b);
        
        VectorXd dg = x.segment<2>(n_state - 3);
        g0 = (g0 + lxly * dg).normalized() * G_NORM;
        //double s = x(n_state - 1);
    }
    
    g = g0;
}

/**
 * 对应 7.3.2节
 * 初始化滑动窗口中每帧的 速度V[0:n] Gravity Vectorg,尺度因子s -> 对应论文的V-B-2
 * 重力修正RefineGravity -> 对应论文的V-B-3
 * 重力方向跟世界坐标的Z轴对齐
 *
 * SolveScale 对应 LinearAlignment
 */
bool SolveScale(map<double, ImageFrame> &all_image_frame,
                Vector3d &g, VectorXd &x)
{
    printf("------ SolveScale ------\n");
    
    int all_frame_count = (int)all_image_frame.size();
    int n_state = all_frame_count * 3 + 3 + 1;
    
    MatrixXd A {n_state, n_state};
    A.setZero();
    VectorXd b {n_state};
    b.setZero();
    
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    
    // 对应 7.3.2节的计算公式
    for (frame_i = all_image_frame.begin();
         next(frame_i) != all_image_frame.end();
         frame_i++, i++)
    {
        frame_j = next(frame_i);
        
        MatrixXd tmp_A(6, 10);
        tmp_A.setZero();
        VectorXd tmp_b(6);
        tmp_b.setZero();
        
        double dt = frame_j->second.pre_integration->sum_dt;
        
        /**
         * tmp_A 对应公式（7-23）的 H
         * x 对应公式（7-24）
         * tmp_b 对应公式（7-25）
         */
        // H_00
        tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        // H_02
        tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
        // H_03
        tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;
        
        Vector3d TIC;
        TIC << TIC_X, TIC_Y, TIC_Z;
        
        // z_00
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC - TIC;
        
        //tmp_b.block<3, 1>(0, 0) = imu_factors[i + 1]->pre_integration.delta_p;
        //cout << "delta_p   " << frame_j->second.imu_factor->pre_integration.delta_p.transpose() << endl;
        
        // H_10
        tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
        // H_11
        tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
        // H_12
        tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity();
        
        // z_10
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        
        //cout << "delta_v   " << frame_j->second.pre_integration.delta_v.transpose() << endl;
        
        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        //MatrixXd cov_inv = cov.inverse();
        cov_inv.setIdentity();
        
        // 对应7.3.2节的公式（7.27），H^T*H ，H^T*b
        MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;
        
        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
        b.segment<6>(i * 3) += r_b.head<6>();
        
        A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
        b.tail<4>() += r_b.tail<4>();
        
        A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
        A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
    }
    
    // AX = b，当求解方程组的系数矩阵是对称矩阵时，用LDLT分解法可以简化程序设计并减少计算量
    // 对应7.3.2节的公式（7.27）
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    
    double s = x(n_state - 1) / 100.0;
    g = x.segment<3>(n_state - 4);
    
    printf("estimated scale: %f\n", s);
    cout << " result g     " << g.norm() << " " << g.transpose() << endl;
    
    if (fabs(g.norm() - G_NORM) > G_THRESHOLD ||  s < 0)
    {
        return false;
    }
    
    // 重力向量优化:进一步细化重力加速度，提高估计值的精度，
    // 形式与LinearAlignment()是一致的，只是将g改为g⋅ĝ+w1b1+w2b2
    RefineGravity(all_image_frame, g, x);   // 在正切空间微调重力向量
    
    s = (x.tail<1>())(0) / 100.0;
    (x.tail<1>())(0) = s;
    
    printf("refine estimated scale: %f", s);
    cout << " refine     " << g.norm() << " " << g.transpose() << endl;
    
    if (s > 0.0)
    {
        printf("vins initial succ!\n");
        return true;
    }
    else
    {
        printf("vins initial fail!!!\n");
        return false;
    }
}

/**
 * cameara与IMU对齐
 * IMU陀螺仪零偏初始化；主要是通过滑动窗口中，每两帧之间通过SFM求解出来的旋转与
 * IMU预积分的旋转量组成一个最小二乘法形式的等式，求解出来陀螺仪的零偏。
 */
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame,
                        Vector3d* Bgs, Vector3d &g, VectorXd &x)
{
    // 估测陀螺仪的Bias，对应论文V-B-1
    // 在滑动窗口中，每两帧之间的相对旋转与IMU预积分产生的旋转进行最小二乘法优化，用ldlt求解
    solveGyroscopeBias(all_image_frame, Bgs);
    
    // 求解速度V、重力向量g、尺度s
    if (SolveScale(all_image_frame, g, x))
    {
        return true;
    }
    else
    {
        return false;
    }
}

