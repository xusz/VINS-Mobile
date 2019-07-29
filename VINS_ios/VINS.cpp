//
//  VINS.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "VINS.hpp"

bool LOOP_CLOSURE = false;  // 闭环处理

VINS::VINS()
      : f_manager{Rs},
        init_fail_cnt{0},
        failure_hand{false},
        drawresult{0.0, 0.0, 0.0, 0.0, 0.0, 7.0}
{
    printf("init VINS begins\n");
    t_drift.setZero();
    r_drift.setIdentity();
    clearState();
    failure_occur = 0;
    last_P.setZero();
    last_R.setIdentity();
    last_P_old.setZero();
    last_R_old.setIdentity();
}

void VINS::setIMUModel()
{
    ProjectionFactor::sqrt_info = FOCUS_LENGTH_X / 1.5 * Matrix2d::Identity();
}

void VINS::clearState()
{
    printf("clear state\n");
    for (int i = 0; i < 10 * (WINDOW_SIZE + 1); i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        IMU_linear[i].setZero();
        IMU_angular[i].setIdentity();
//        pre_integrations[i] = nullptr;
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();
        
        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
            pre_integrations[i] = nullptr;
        }
//        pre_integrations[i] = nullptr;
    }
    
    tic << TIC_X, TIC_Y, TIC_Z;
    ric = Utility::ypr2R(Vector3d(RIC_y, RIC_p, RIC_r));
    
    frame_count = 0;
    first_imu = false;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    initProgress = 0;
    
    if (tmp_pre_integration != nullptr)
    {
        delete tmp_pre_integration;
        tmp_pre_integration = nullptr;
    }
    if (last_marginalization_info != nullptr)
    {
        delete last_marginalization_info;
        last_marginalization_info = nullptr;
    }
    
//    tmp_pre_integration = nullptr;
//    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();
    
    f_manager.clearState();
}

void VINS::setExtrinsic()
{
    tic << TIC_X, TIC_Y, TIC_Z;
    ric = Utility::ypr2R(Vector3d(RIC_y, RIC_p, RIC_r));
}

// 数据转换，因为ceres使用数值数组
// （猜测）将PVQ等赋值给待优化参数para_*，在ceres优化过程中作为初始值
void VINS::old2new()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();
        
        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();
        
        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();
        
        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic.x();
        para_Ex_Pose[i][1] = tic.y();
        para_Ex_Pose[i][2] = tic.z();
        
        Quaterniond q{ric};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }
    
    // triangulate
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
    {
        para_Feature[i][0] = dep(i);
    }
}

/**
 * 数据转换，因为ceres使用数值数组，vector2double的相反过程
 */
// （猜测）ceres将待优化参数para_*优化完毕后，赋值给PVQ等
void VINS::new2old()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];
    
    if (failure_occur)
    {
        printf("failure recover %lf %lf %lf\n", last_P.x(), last_P.y(), last_P.z());
        origin_R0 = Utility::R2ypr(last_R_old);
        origin_P0 = last_P_old;
    }
    
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5]).toRotationMatrix());
    
    double y_diff = origin_R0.x() - origin_R00.x();
    // TODO:
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6],
                                       para_Pose[i][3],
                                       para_Pose[i][4],
                                       para_Pose[i][5]).normalized().toRotationMatrix();
        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);
        
        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);
        
        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }
    
    Vector3d cur_P0 = Ps[0];
    
    if (LOOP_CLOSURE && loop_enable)
    {
        loop_enable = false;
        for (int i = 0; i< WINDOW_SIZE; i++)
        {
            if (front_pose.header == Headers[i])
            {
                Matrix3d Rs_loop = Quaterniond(front_pose.loop_pose[6],
                                               front_pose.loop_pose[3],
                                               front_pose.loop_pose[4],
                                               front_pose.loop_pose[5]).normalized().toRotationMatrix();
                Vector3d Ps_loop = Vector3d(front_pose.loop_pose[0],
                                            front_pose.loop_pose[1],
                                            front_pose.loop_pose[2]);
                
                Rs_loop = rot_diff * Rs_loop;
                Ps_loop = rot_diff * (Ps_loop - Vector3d(para_Pose[0][0],
                                                         para_Pose[0][1],
                                                         para_Pose[0][2]))
                            + origin_P0;
                
                double drift_yaw = Utility::R2ypr(front_pose.Q_old.toRotationMatrix()).x()
                                    - Utility::R2ypr(Rs_loop).x();
                
                r_drift = Utility::ypr2R(Vector3d(drift_yaw, 0, 0));
                //r_drift = front_pose.Q_old.toRotationMatrix() * Rs_loop.transpose();
                t_drift = front_pose.P_old - r_drift * Ps_loop;
            }
        }
    }
    
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic = Vector3d(para_Ex_Pose[i][0],
                       para_Ex_Pose[i][1],
                       para_Ex_Pose[i][2]);
        
        ric = Quaterniond(para_Ex_Pose[i][6],
                          para_Ex_Pose[i][3],
                          para_Ex_Pose[i][4],
                          para_Ex_Pose[i][5]).toRotationMatrix();
    }
    
    // 赋值λ
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
    {
        dep(i) = para_Feature[i][0];
    }
    f_manager.setDepth(dep);
}

/**
 * 检测SLAM系统是否失败
 */
bool VINS::failureDetection()
{
    bool is_failure = false;
    
    if (f_manager.last_track_num < 4)
    {
        printf("failure little feature %d\n", f_manager.last_track_num);
        is_failure = true;
    }
    /*
     if (Bas[WINDOW_SIZE].norm() > 1)
     {
     printf("failure  big IMU acc bias estimation %f\n", Bas[WINDOW_SIZE].norm());
     is_failure = true;
     }
     */
    if (Bgs[WINDOW_SIZE].norm() > 1)
    {
        printf("failure  big IMU gyr bias estimation %f\n", Bgs[WINDOW_SIZE].norm());
        is_failure = true;
    }
    
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 1)
    {
        printf("failure big translation\n");
        is_failure = true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 0.5)
    {
        printf("failure  big z translation\n");
        is_failure = true;
    }
    
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
   
    if (delta_angle > 40)
    {
        printf("failure  big delta_angle \n");
        is_failure = true;
    }
    
    if (failure_hand)
    {
        failure_hand = false;
        is_failure = true;
        printf("failure by hand!\n");
    }
    
    return is_failure;
}

/*
 void VINS::failureRecover()
 {
 int his_index = 0;
 for(int i = 0; i < WINDOW_SIZE; i++)
 {
 if(Headers_his[i] == Headers[0])
 {
 his_index = i;
 break;
 }
 if(i == WINDOW_SIZE -1)
 his_index = i;
 }
 Vector3d his_R0 = Utility::R2ypr(Rs_his[his_index]);
 
 Vector3d his_P0 = Ps_his[his_index];
 
 Vector3d cur_R0 = Utility::R2ypr(Rs[0]);
 Vector3d cur_P0 = Ps[0];
 
 double y_diff = his_R0.x() - cur_R0.x();
 
 Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
 
 for (int i = 0; i <= WINDOW_SIZE; i++)
 {
 Rs[i] = rot_diff * Rs[i];
 Ps[i] = rot_diff * (Ps[i] - cur_P0) + his_P0;
 Vs[i] = rot_diff * Vs[i];
 }
 }
 */

void VINS::reInit()
{
    failure_hand = true;
    failureDetection();
}

void VINS::update_loop_correction()
{
    // update loop correct pointcloud
    correct_point_cloud.clear();
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = (int)it_per_id.feature_per_frame.size();
        
        if (!(it_per_id.used_num >= 4 && it_per_id.start_frame < WINDOW_SIZE - 2))
        {
            continue;
        }
        
        if (/*it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 ||*/ it_per_id.solve_flag != SOLVE_SUCC)
        {
            continue;
        }
        
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d tmp = r_drift * Rs[imu_i] * (ric * pts_i + tic) + r_drift * Ps[imu_i] + t_drift;
        correct_point_cloud.push_back(tmp.cast<float>());
    }
    
    // update correct pose
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        Vector3d correct_p = r_drift * Ps[i] + t_drift;
        correct_Ps[i] = correct_p.cast<float>();
        Matrix3d correct_r = r_drift * Rs[i];
        correct_Rs[i] = correct_r.cast<float>();
    }
}

/**
 * 1 当滑窗不满的时候，把当前测量值加入到滑窗指定位置
 * 2 残差和雅可比矩阵、协方差矩阵保存在pre_integrations中
 *      propagate (_dt[时间间隔]; _acc_1 [加速度计数据]; _gyr_1 [陀螺仪数据])
 *      里面主要为 midPointIntegration 函数即:   中值法进行预积分
 *      得到状态变化量result_delta_q，result_delta_p，result_delta_v，
 *      result_linearized_ba，result_linearized_bg和得到更新的协方差矩阵和雅可比矩阵
 * 3 提供imu计算的当前旋转，位置，速度，作为优化的初值
 *
 * @param dt 是当前IMU和前一个IMU的时间差
 * @param linear_acceleration 为当前加速度计数据
 * @param angular_velocity 为当前角速度计数据
 */
void VINS::processIMU(double dt,
                      const Vector3d &linear_acceleration,
                      const Vector3d &angular_velocity)
{
    // clearState() 后 first_imu = false
    if (!first_imu)  // 未获取第一帧IMU数据
    {
        first_imu = true;
        
        // 将第一帧IMU数据记录下来
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    
    // 调用imu的预积分，计算对应的状态量、协方差和雅可比矩阵；注意frame_count参数的作用
    // 用中值法进行预积分
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0,
                                                        Bas[frame_count],
                                                        Bgs[frame_count]};
    }
    
    // 每次pre_integration 推入一个IMU数据后就会进行积分：
    if (frame_count != 0) // 在初始化时，第一帧图像特征点数据没有对应的预积分
    {
        // covariance propagate  协方差传播
        pre_integrations[frame_count]->push_back(dt,
                                                 linear_acceleration,
                                                 angular_velocity);
        
        // comments because of recovering
        if (solver_flag != NON_LINEAR)  // 还未初始化完成
        {
            tmp_pre_integration->push_back(dt,
                                           linear_acceleration,
                                           angular_velocity);
        }
        
        // 将时间、加速度和角速度分别存入相应的缓存中
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);
        
        /**
         * IMU数据进行积分，得到当前时刻的PVQ，使用midpoint integration，对应5.3节
         * 当积完一个measurement中所有IMU数据后，就得到了对应frame在[世界坐标系]中的Ps、Vs、Rs
         *
         * 下面这部分的积分，在未完成初始化时似乎是没有意义的，因为未完成初始化时，对IMU数据来说是没有世界坐标系的
         * 当完成初始化后，下面的积分才有用，它可以通过IMU积分得到滑动窗口中最新帧在世界坐标系中的PVQ
         */
        {
            Vector3d g {0, 0, GRAVITY};
            int j = frame_count;
            // 下面都采用的是中值积分的传播方式， noise是zero mean Gassu，在这里忽略了
            
            // 对应5.3节中的公式（5-13）的前半部分
            Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
            
            // 对应5.3节中的公式（5-14）
            Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
            // 各帧在世界坐标系中的旋转，对应5.3节中的公式（5-12）
            Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            
            // 对应5.3节中的公式（5-13）的后半部分
            Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
            // 对应5.3节中的公式（5-13）
            Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
            
            // 各帧在世界坐标系中的位移，对应5.3节中的公式（5-10）
            Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
            // 各帧在世界坐标系中的速度，对应5.3节中的公式（5-11）
            Vs[j] += dt * un_acc;
        }
    }
    
    // 保留此次接收到的IMU数据，用于下一次IMU数据积分
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

/*
 * processImage
 * 当处理完一批imu_msg后，在process函数中就会紧接着处理图像数据,
 * 当图像数量达到窗口大小时，在solve_ceres函数中就会把IMU误差项加进去进行优化，
 * 在这之前有一些边缘化的操作，而且这个操作会影响pre_integrations数组
 * image_msg -- point_clouds，当前帧的点云数据
 */
void VINS::processImage(map<int, Vector3d> &image_msg,
                        double header,
                        int buf_num)
{
    int track_num = 0;  // 相邻帧跟踪到的特征点数
    printf("adding feature points %lu\n", image_msg.size());
    
    // 对相近的特征点进行视差计算
    /* 通过检测两帧之间的视差决定是否作为关键帧，
     * 同时添加之前检测到的特征点到feature（list< FeaturePerId >）这个容器中，
     * 计算每一个点跟踪的次数，以及它的视差
     */
    // 通过检测两帧之间的视差决定是否作为关键帧，同时添加之前检测到的特征点到feature容器中，
    // 计算每一个点跟踪的次数，以及它的视差
    /*
     * 把当前帧图像（frame_count）的特征点添加到f_manager.feature容器中
     * 计算第2最新帧与第3最新帧之间的平均视差（当前帧是第1最新帧），
     *      然后判断是否把第2最新帧添加为关键帧
     * 在未完成初始化时，如果窗口没有塞满，那么是否添加关键帧的判定结果不起作用，滑动窗口要塞满
     * 只有在滑动拆个纽扣塞满后，或者初始化完成之后，才需要滑动窗口，此时才需要做关键帧判别，
     *      根据第2最新关键帧是否未关键帧选择相应的边缘化策略
     */
    // 向Featuresmanger中添加Features并确定共视关系及视差角的大小
    /** 判断该帧是否关键帧
     * 关键帧的判断依据是rotation-compensated过后的parallax足够大，
     * 并且tracking上的feature足够多；关键帧会保留在当前Sliding Window中，
     * marginalize掉Sliding Window中最旧的状态，如果是非关键帧则优先marginalize掉
     */
    if (f_manager.addFeatureCheckParallax(frame_count, image_msg, track_num))
    {
        marginalization_flag = MARGIN_OLD;
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
    }
    
    // printf("marginalization_flag %d\n", int(marginalization_flag));
    // printf("this frame is-------------------------------%s\n",
    //          marginalization_flag ? "reject" : "accept");
    // printf("Solving %d\n", frame_count);
//    printf("number of feature: %d %d\n",
//           feature_num = f_manager.getFeatureCount(), track_num);
    
    Headers[frame_count] = header;
    
    if (solver_flag == INITIAL)  // 需要初始化，进行线性初始化
    {
        ImageFrame imageframe(image_msg, header);
        imageframe.pre_integration = tmp_pre_integration;
        // all_image_frame:for initialization 存储相应的预积分和图像特征点信息
        all_image_frame.insert(make_pair(header, imageframe));

        tmp_pre_integration = new IntegrationBase{acc_0, gyr_0,
                                                Vector3d(0,0,0),
                                                Vector3d(0,0,0)};
        // 滑动窗口中塞满了才进行初始化
        if (frame_count == WINDOW_SIZE)
        {
            if (track_num < 20)
            {
                clearState();
                return;
            }
            
            bool result = false;
            
            if (header - initial_timestamp > 0.3)
            {
                // vins系统初始化
                result = solveInitial();
                initial_timestamp = header;
            }
            
            if (result)  // 初始化成功
            {
                /**
                 * 在solve_ceres函数中就会把IMU误差项加进去进行优化，
                 * 在这之前有一些边缘化的操作，而且这个操作会影响pre_integrations数组
                 */
                solve_ceres(buf_num);
                // initialization failed, need reinitialize
                if (final_cost > 3000)  // if (final_cost > 200) TODO:
                {
                    printf("------------- Initialization Fail ! -------------\n");
                    printf("final cost %lf faild!\n", final_cost);
                    
                    delete last_marginalization_info;
                    last_marginalization_info = nullptr;
                    solver_flag = INITIAL;
                    init_status = FAIL_COST;
                    init_fail_cnt++;
                    slideWindow();
                }
                else
                {
                    printf("final cost %lf succ!\n", final_cost);
                    failure_occur = 0;
                    //update init progress
                    initProgress = 100;
                    init_status = SUCC;
                    init_fail_cnt = 0;
                    
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    f_manager.removeFailures();
                    update_loop_correction();
                    last_R = Rs[WINDOW_SIZE];
                    last_P = Ps[WINDOW_SIZE];
                    last_R_old = Rs[0];
                    last_P_old = Ps[0];
                    
                    printf("------------- Initialization finish ! -------------\n");
                }
            }
            else
            {
                slideWindow();  // 初始化不成功，对窗口进行滑动
            }
        }
        else
        {
            frame_count++;   // 滑动窗口没塞满，接着塞
            initProgress += 2;
        }
    }
    else  // solver_flag == NON_LINEAR 进行非线性优化
    {
        // 已经成功初始化，进行正常的VIO紧耦合优化
        bool is_nonlinear = true;
        // TODO: 作用？？？
        f_manager.triangulate(Ps, tic, ric, is_nonlinear);
        
        /**
         * 后端非线性优化
         * 在solve_ceres函数中就会把IMU误差项加进去进行优化，
         * 在这之前有一些边缘化的操作，而且这个操作会影响pre_integrations数组
         */
        solve_ceres(buf_num);
        
        failure_occur = 0;
        // 失效检测，如果失效则重启VINS系统
        if (failureDetection())
        {
            failure_occur = 1;
            clearState();
            return;
        }
        
        slideWindow();
        
        f_manager.removeFailures();
        update_loop_correction();
        
        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R_old = Rs[0];
        last_P_old = Ps[0];
    }
}

/**
 * 基于滑动窗口的紧耦合的后端非线性优化，对应第八章
 *
 * 在solve_ceres函数中就会把IMU误差项加进去进行优化，
 * 在这之前有一些边缘化的操作，而且这个操作会影响pre_integrations数组
 */
// 1. 添加要优化的变量，也就是滑动窗口的位置para_Pose[0:n]
//    速度和Bias para_SpeedBias[0:n]一共15个自由度，IMU的外参也可以加进来估计
// 2. 添加残差，残差项分为4块 先验残差+IMU残差+视觉残差+闭环检测的残差
// 3. 根据倒数第二帧是不是关键帧确定marginization的结果，下面有详细解释
// void Estimator::optimization()
//
// 16.9节 后端优化之optimazation
void VINS::solve_ceres(int buf_num)
{
    ceres::Problem problem;
    // loss_function = new ceres::HuberLoss(1.0);
    //！设置柯西损失函数因子
    ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
    
    //！Step1.1：添加待优化状态量 [p,q](7)，[speed,ba,bg](9)
    // 添加sliding window frame的state，(pose,v,q,ba,bg),
    // 因为ceres用的是double数组，所以在下面用vector2double做类型转换，
    // 把原来的Ps Vs Bgs Bas转到para_Pose para_SpeedBias下
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    
    //！Step1.2：添加camera与IMU的外参[p_cb,q_cb](7)
    // camera IMU的外参也添加到估计
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        // 令指定的参数块在整个优化计算过程中保持常数
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
    
    for (int i = 0; i < NUM_OF_F; i++)
    {
        problem.AddParameterBlock(para_Feature[i], SIZE_FEATURE);
    }
    
    // 数据类型转换，Ps,Rs 转换为 para_Pose
    // Vs,Bas,Bgs 转换为 para_SpeedBias
    // tic,ric 转换为 para_Ex_Pose。
    old2new();
    
    // marginalization factor 边缘化因子
    // 添加marginalization的residual，这个marginalization的结构是始终存在的，
    // 随着下面marginazation的结构更新，last_marginalization_parameter_blocks
    // 对应的是还在sliding window的变量
    // 关于这一部分的理解http://blog.csdn.net/heyijia0327/article/details/53707261
    // 这里可以这样理解，下面会添加对IMU和视觉的残差，但是，这些对应的变量实际上跟之前
    // 被margin掉的变量是有约束的，这里的last_marginalization_parameter_blocks
    // 就是保存的这些变量，也就是heyijia博客中对应的Xb变量，
    // last_marginalization_info中对应的是Xb对应的测量Zb，
    // 这里用先验来表示这个约束，整个margin部分实际上就是在维护这个结构：
    /**
     * add residual for prior from last marginalization
     * 添加边缘化残差约束：1个
     */
    //！Step1.3：添加滑窗残差约束
    if (last_marginalization_info != nullptr)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    
    // 一个IMUfactor误差项对应着一个pre_integration实例，而且计算的是两个帧之间IMU的误差
    // 这里IMU项和camera项之间是有一个系数，这个系数就是它们各自的协方差矩阵：IMU的协方差
    // 是预积分的协方差(IMUFactor::Evaluate，中添加IMU协方差，求解jacobian矩阵)，
    // 而camera的则是一个固定的系数（f/1.5）
    /**
     * 添加IMU残差约束：WINDOW_SIZE个, 每相邻两个Pose之间一个IMU residual项
     */
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;   // pre_integrations[ i+1 ]
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL,
                                 para_Pose[i],
                                 para_SpeedBias[i],
                                 para_Pose[j],
                                 para_SpeedBias[j]);
    }
    
    // projection factor 视觉投影因子
    int f_m_cnt = 0;
    double f_sum = 0.0;
    double r_f_sum = 0.0;
    int feature_index = -1;
    
    /**
     * ！Step1.4：添加视觉残差约束，add residual for per feature to per frame
     * 添加视觉残差约束：被观测数大于2的特征, 首次观测与后面的每次观测之间各一个residual项
     */
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = (int)it_per_id.feature_per_frame.size();
        // 此特征点要至少被2帧图像观测到 && 被第一次观测到的帧不能是倒数2帧以后的帧
        if (!it_per_id.isPriorFeature())
        {
            continue;
        }
        
        ++feature_index;
        
        int imu_i = it_per_id.start_frame;
        int imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;
            
            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_i],
                                     para_Pose[imu_j],
                                     para_Ex_Pose[0],
                                     para_Feature[feature_index]);
            
            f_m_cnt++;
            
            double **para = new double *[4];
            para[0] = para_Pose[imu_i];
            para[1] = para_Pose[imu_j];
            para[2] = para_Ex_Pose[0];
            para[3] = para_Feature[feature_index];
            double *res = new double[2];
            
            // 计算残差
            f->Evaluate(para, res, NULL);
            f_sum += sqrt(res[0] * res[0] + res[1] * res[1]);
            
            double rho[3];
            loss_function->Evaluate(res[0] * res[0] + res[1] * res[1], rho);
            r_f_sum += rho[0];
        }
    }
    
    visual_cost = r_f_sum;
    visual_factor_num = f_m_cnt;
    
    // 添加闭环的参数和residual
    if (LOOP_CLOSURE)
    {
        // loop close factor
        // front_pose.measurements.clear();
        if (front_pose.header != retrive_pose_data.header)
        {
            front_pose = retrive_pose_data;  // need lock
            printf("use loop\n");
        }
        
        if (!front_pose.measurements.empty())
        {
            // the retrive pose is in the current window
            if (front_pose.header >= Headers[0])
            {
                // tmp_retrive_pose_buf.push(front_pose);
                printf("loop front pose  in window\n");
                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    //！建立闭环约束
                    if (front_pose.header == Headers[i])
                    {
                        for (int k = 0; k < 7; k++)
                        {
                            front_pose.loop_pose[k] = para_Pose[i][k];
                        }
                        
                        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
                        problem.AddParameterBlock(front_pose.loop_pose,
                                                  SIZE_POSE,
                                                  local_parameterization);
                        
                        int retrive_feature_index = 0;
                        int feature_index = -1;
                        int loop_factor_cnt = 0;
                        
                        //！遍历滑窗内的特征点
                        for (auto &it_per_id : f_manager.feature)
                        {
                            it_per_id.used_num = (int)it_per_id.feature_per_frame.size();
                            //！至少有两帧图像观测到该特征点且不是滑窗内的最后两帧
                            if (!it_per_id.isPriorFeature())
                            {
                                continue;
                            }
                            
                            ++feature_index;
                            
                            int start = it_per_id.start_frame;
                            // feature has been obeserved in ith frame
                            int end = (int)(start + it_per_id.feature_per_frame.size() - i - 1);
                           
                            //！如果该Feature是被滑窗中本帧之前观测到
                            if (start <= i && end >= 0)
                            {
                                while (front_pose.features_ids[retrive_feature_index] < it_per_id.feature_id)
                                {
                                    retrive_feature_index++;
                                }
                                
                                //！将拥有固定位姿的闭环帧加入到Visual-Inertail BA中
                                if (front_pose.features_ids[retrive_feature_index] == it_per_id.feature_id)
                                {
                                    Vector3d pts_j = Vector3d(front_pose.measurements[retrive_feature_index].x,
                                                              front_pose.measurements[retrive_feature_index].y,
                                                              1.0);
                                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                                    // double ratio = 1.0;
                                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                                    problem.AddResidualBlock(f, loss_function,
                                                             para_Pose[start],
                                                             front_pose.loop_pose,
                                                             para_Ex_Pose[0],
                                                             para_Feature[feature_index]);
                                    
                                    // printf("loop add factor %d %d %lf %lf %d\n",
                                    //        retrive_feature_index, feature_index,
                                    //        pts_j.x(), pts_i.x(), front_pose.features_ids.size());
                                    
                                    retrive_feature_index++;
                                    loop_factor_cnt++;
                                    loop_enable = true;
                                }
                            }
                        }
                        
                        printf("add %d loop factor\n", loop_factor_cnt);
                    }
                }
            }
        }
    }
    
    // 设置 ceres 属性
    ceres::Solver::Options options;
    
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 1;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = false;
    // # max solver itrations, to guarantee real time
    options.max_num_iterations = 10;
    // options.use_nonmonotonic_steps = true;
    
    // 最大求解时间
    if (buf_num < 2)
    {
        options.max_solver_time_in_seconds = SOLVER_TIME;
    }
    else if (buf_num < 4)
    {
        options.max_solver_time_in_seconds = SOLVER_TIME * 2.0 / 3.0;
    }
    else  // >= 4
    {
        options.max_solver_time_in_seconds = SOLVER_TIME / 2.0;
    }
    
    ceres::Solver::Summary summary;
    //TE(prepare_solver);
    /*
     * IMU误差的Jacobian矩阵的计算,
     * 这里就使用到了pre_integration实例里面的Jacobian的部分结果，
     * Jacobian数组里每一项都是IMU误差关于两帧图像状态的导数，只不过这里把pose和speedBias分开了
     */
    TS(ceres);
//    printf("ceres::Solve \n");
    // 约束求解
    ceres::Solve(options, &problem, &summary);
    final_cost = summary.final_cost;
    // cout << summary.FullReport() << endl;
    TE(ceres);
    
    /*****************优化后的内容********************/
    //！求解两个闭环帧之间的关系
    // relative info between two loop frame
    if (LOOP_CLOSURE)
    {
        for (int i = 0; i< WINDOW_SIZE; i++)
        {
            //！闭环检测成功
            if (front_pose.header == Headers[i])
            {
                // 四元数
                Matrix3d Rs_i = Quaterniond(para_Pose[i][6],
                                            para_Pose[i][3],
                                            para_Pose[i][4],
                                            para_Pose[i][5]).normalized().toRotationMatrix();
                
                Vector3d Ps_i = Vector3d(para_Pose[i][0],
                                         para_Pose[i][1],
                                         para_Pose[i][2]);
                
                Matrix3d Rs_loop = Quaterniond(front_pose.loop_pose[6],
                                               front_pose.loop_pose[3],
                                               front_pose.loop_pose[4],
                                               front_pose.loop_pose[5]).normalized().toRotationMatrix();
                
                Vector3d Ps_loop = Vector3d(front_pose.loop_pose[0],
                                            front_pose.loop_pose[1],
                                            front_pose.loop_pose[2]);
                
                front_pose.relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
                front_pose.relative_q = Rs_loop.transpose() * Rs_i;
                front_pose.relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x()
                                                                - Utility::R2ypr(Rs_loop).x());
            }
        }
    }
    
    // （猜测）ceres::Solve 将待优化参数para_* 优化完毕后，赋值给PVQ等
    new2old();
    
    vector<ceres::ResidualBlockId> residual_set;
    problem.GetResidualBlocks(&residual_set);
    for (auto it : residual_set)
    {
        problem.RemoveResidualBlock(it);
    }
    
    /**
     * Step3:marg部分
     *  3.1 对于边缘化首帧
     *    3.1.1 把之前存的残差部分加进来
     *    3.1.2 把与首帧相关的残差项加进来,包含IMU,vision.
     *    3.1.3 计算所有残差项的残差和雅克比
     *    3.1.4 多线程构造Hx=b的结构
     *    3.1.5 marg结束,调整参数块在下一次window的位置
     *  3.2 对于边缘化倒数第二帧
     *    3.2.1 如果倒数第二帧不是关键帧,保留该帧的IMU测量,去掉该帧的visual,代码中都没写.
     *    3.2.2 计算所有残差项的残差和雅克比
     *    3.2.3 多线程构造Hx=b的结构,(需要细看)
     *    3.2.4 marg结束,调整参数块在下一次window的位置
     */
    // for marginalization back
    // margin部分，如果倒数第二帧是关键帧：
    // 1.把之前存的残差部分加进来
    // 2.把与当前要margin掉帧所有相关的残差项都加进来，IMU,vision
    // 3.preMarginalize-> 调用Evaluate计算所有ResidualBlock的残差，parameter_block_data parameter_block_idx parameter_block_size是marinazation中存参数块的容器(unordered_map),key都是addr,
    //分别对应这些参数的data，在稀疏矩阵A中的index(要被margin掉的参数会被移到前面)，A中的大小
    // 4.Marginalize->多线程构造Hx=b的结构，H是边缘化后的结果，First Esitimate Jacobian,在X_0处线性化
    // 5.margin结束，调整参数块在下一次window中对应的位置（往前移一格），
    // 注意这里是指针，后面slideWindow中会赋新值，这里只是提前占座（知乎上有人问：
    // https://www.zhihu.com/question/63754583/answer/259699612
    if (marginalization_flag == MARGIN_OLD)  // marg 最老帧
    {
        /**
         *  3.1 对于边缘化首帧
         *     3.1.1 把之前存的残差部分加进来；
         *     3.1.2 把与首帧相关的残差项加进来，包含IMU、vision；
         *     3.1.3 计算所有残差项的残差和雅克比；
         *     3.1.4 多线程构造 Hx=b 的结构（需要细看）；
         *     3.1.5 marg结束，调整参数块在下一次window的位置；
         */
        // 向ResidualBlockInfo容器中(factors)添加先验残差、最新IMU测量残差、camera所有特征点测量残差
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        
        old2new();
        
        //! 先验误差会一直保存，而不是只使用一次
        //! 如果上一次边缘化的信息存在，要边缘化的参数块是 para_Pose[0] para_SpeedBias[0]
        //      以及 para_Feature[feature_index](滑窗内的第feature_index个点的逆深度)
        // 1. 把上一次先验项中的残差项(尺寸为 n)传递给当前先验项，并从中去除需要丢弃的状态量
        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                //！查询last_marginalization_parameter_blocks中是首帧状态量的序号
                if (last_marginalization_parameter_blocks[i] == para_Pose[0]
                    || last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                {
                    drop_set.push_back(i);
                }
            }
            
            //! 构造边缘化的的Factor
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            
            //! 添加上一次边缘化的参数块
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                                                marginalization_factor,
                                                NULL,
                                                last_marginalization_parameter_blocks,
                                                drop_set);
            
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        
        // 2. 将滑窗内第0帧和第1帧间的IMU预积分因子(pre_integrations[1])放到 marginalization_info 中，
        //    即图中上半部分中x0和x1之间的表示IMU约束的黄色块
        //！添加IMU的先验，只包含边缘化帧的IMU测量残差
        //！Question：不应该是pre_integrations[0]么
        //!
        //// if (pre_integrations[1]->sum_dt < 10.0)
        {
            // 需要Marg的是最老帧，所以是第0帧与第1帧之间的pre_integrations
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
            // dropset{0, 1} 对应marg帧的Pose、SpeedBias
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                                                        imu_factor,
                                                        NULL,
                                                        vector<double *>{para_Pose[0],
                                                                         para_SpeedBias[0],
                                                                         para_Pose[1],
                                                                         para_SpeedBias[1]},
                                                        vector<int>{0, 1} );
            
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        
        // 3. 挑选出第一次观测帧为第0帧的路标点，将对应的多组视觉观测放到 marginalization_info 中，
        //    即图中上半部分中x0所看到的红色五角星的路标点
        //！添加视觉的先验，只添加起始帧是旧帧且观测次数大于2的Features
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = (int)it_per_id.feature_per_frame.size();
                
                // 此特征点要至少被2帧图像观测到 && 被第一次观测到的帧不能是倒数2帧以后的帧
                if (!it_per_id.isPriorFeature())
                {
                    continue;
                }
                
                ++feature_index;
                
                int imu_i = it_per_id.start_frame;
                int imu_j = imu_i - 1;
                
                //! 只选择被边缘化的帧的Features
                // 找在第0帧观测到的特征点
                // TODO: 移动滑动窗口后，frameid是否会被更新，从0开始重新计数？
                if (imu_i != 0)
                {
                    continue;
                }
                
                //! 得到该Feature在起始观测帧下的归一化坐标
                Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                
                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    //! 和起始观测帧是同一帧
                    if (imu_i == imu_j) {
                        continue;
                    }
                    
                    Vector3d pts_j = it_per_frame.point;
                    ProjectionFactor *cost_func = new ProjectionFactor(pts_i, pts_j);
                    
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                                                cost_func,
                                                loss_function,
                                                vector<double *>{para_Pose[imu_i],
                                                                para_Pose[imu_j],
                                                                para_Ex_Pose[0],
                                                                para_Feature[feature_index]},
                                                vector<int>{0, 3});
                    
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }
        }
        
        //! 将三个ResidualBlockInfo中的参数块综合到marginalization_info中
        //  计算所有ResidualBlock(残差项)的残差和雅克比,parameter_block_data是参数块的容器
        
        TS(per_marginalization);
        // 根据各个测量模型Evaluate() 计算残差；
        // 各个参数块拷贝到统一的内存（parameter_block_data）中
        // 4. 得到IMU和视觉观测(cost_function)对应的参数块(parameter_blocks)，雅可比矩阵，残差值(residuals)
        marginalization_info->preMarginalize();
        TE(per_marginalization);
        
        TS(marginalization);
        // 5. 多线程计整个先验项的参数块，雅可比矩阵和残差值，对应舒尔补公式（9-7）
        marginalization_info->marginalize();
        TE(marginalization);
        
        /**
         * 6. 最后移交了优化项需要得到的两个变量：last_marginalization_info和last_marginalization_parameter_blocks
         */
        //！将滑窗里关键帧位姿移位，为什么是向右移位了呢？
        //! 这里是保存了所有状态量的信息，为什么没有保存逆深度的状态量呢
        std::unordered_map<long, double *> addr_shift;
        
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            // TODO: what's the meaning
            // 可以把一个指针转换成一个整数，也可以把一个整数转换成一个指针
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        }
        
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
        if (last_marginalization_info)
        {
            delete last_marginalization_info;
        }
        
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else  // marginalization_flag == MARGIN_SECOND_NEW  marg次新帧
    {
        /**
         *  3.2 对于边缘化倒数第二帧
         *      3.2.1 如果倒数第二帧不是关键帧，保留该帧的IMU测量，去掉该帧的visual；代码中都没写；
         *      3.2.2 计算所有残差项的残差和雅克比；
         *      3.2.3 多线程构造 Hx=b 的结构（需要细看）；
         *      3.2.4 marg结束，调整参数块在下一次window的位置。
         */
        //！边缘化倒数第二帧，如果倒数第二帧不是关键帧
        // 1.保留该帧的IMU测量,去掉该帧的visual,代码中都没有写.
        // 2.premarg
        // 3.marg
        // 4.滑动窗口移动
        
        // 如果倒数第二帧不是关键帧，则把这帧的视觉测量舍弃掉（边缘化）但保留IMU测量值在滑动窗口中。（其它步骤和上一步骤相同）
        // 1.保留该帧的IMU测量，margin该帧的visual
        // 2.premargin
        // 3.marginalize
        // 4.滑动窗口移动（去掉倒数第二个）
        if (last_marginalization_info
            && std::count(std::begin(last_marginalization_parameter_blocks),
                          std::end(last_marginalization_parameter_blocks),
                          para_Pose[WINDOW_SIZE - 1]))
        {
            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            
            old2new();
           
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    assert(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    //！寻找导数第二帧的位姿
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                    {
                        drop_set.push_back(i);
                    }
                }
                
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                                                    marginalization_factor,
                                                    NULL,
                                                    last_marginalization_parameter_blocks,
                                                    drop_set);
                
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
            
            marginalization_info->preMarginalize();
            marginalization_info->marginalize();
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                {
                    continue;
                }
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            }
            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
            {
                delete last_marginalization_info;
            }
            
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
}

/**
 * vins系统初始化
 * 1.确保IMU有足够的excitation
 * 2.检查当前帧（滑动窗口中的最新帧）与滑动窗口中所有图像帧之间的特征点匹配关系，
 *   选择跟当前帧中有足够多数量的特征点（30个）被跟踪，且由足够视差（20 pixels）的某一帧，利用五点法恢复相对旋转和平移量。
 *   如果找不到，则在滑动窗口中保留当前帧，然后等待新的图像帧
 * 3.sfm.construct 全局SFM 恢复滑动窗口中所有帧的位姿，以及特特征点三角化
 * 4.利用pnp恢复其他帧
 * 5.visual-inertial alignment：视觉SFM的结果与IMU预积分结果对齐
 * 6.给滑动窗口中要优化的变量一个合理的初始值以便进行非线性优化
 */
bool VINS::solveInitial()
{
    printf("-------------------- solve initial --------------------\n");
    printf("PS %lf %lf %lf\n", Ps[0].x(), Ps[0].y(), Ps[0].z());
    // check imu observibility, 通过协方差检测IMU的可观测性
    // 保证IMU充分运动，通过线加速度判断，一开始通过线加速度的标准差（离散程度）判断保证IMU充分运动，加速度标准差大于0.25则代表imu充分激励，足够初始化。
    // TODO: 为什么注释掉？？？
    /*
     {
     map<double, ImageFrame>::iterator frame_it;
     Vector3d sum_g;
     for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
     {
     double dt = frame_it->second.pre_integration->sum_dt;
     if(dt == 0)
     {
     printf("init IMU variation not enouth!\n");
     init_status = FAIL_IMU;
     init_fail_cnt++;
     return false;
     }
     Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
     sum_g += tmp_g;
     }
     
     Vector3d aver_g;
     aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
     cout << "aver_g " << aver_g.transpose() << endl;
     double var = 0;
     for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
     {
     double dt = frame_it->second.pre_integration->sum_dt;
     Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
     var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
     //cout << "frame g " << tmp_g.transpose() << endl;
     }
     var = sqrt(var / ((int)all_image_frame.size() - 1));
     printf("IMU variation %f!\n", var);
     if(var < 0.25)
     {
     printf("init IMU variation not enouth!\n");
     init_status = FAIL_IMU;
     init_fail_cnt++;
     return false;
     }
     }
     */
    
    // global sfm
    // 滑动窗口中每一帧的姿态，旋转四元数
    Quaterniond Q[frame_count + 1];
    // 滑动窗口中每一帧的位置
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    
    {
        // 特征点对应的3D feature
        // 用于视觉初始化的图像特征点数据
        for (auto &it_per_id : f_manager.feature)
        {
            int imu_j = it_per_id.start_frame - 1;
            
            SFMFeature tmp_feature;
            tmp_feature.state = false;  // 该特征点的初始状态为：未被三角化
            tmp_feature.id = it_per_id.feature_id;
            
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;   // 观测到该特征点的图像帧的帧号
                Vector3d pts_j = it_per_frame.point;
                tmp_feature.observation.push_back(make_pair(imu_j,
                                    Eigen::Vector2d{pts_j.x(), pts_j.y()} ));
            }
            sfm_f.push_back(tmp_feature);
        }
        
        /**
         * 在窗口内选择跟最后一帧视差最大的帧，利用五点法计算相对旋转和平移量
         */
        Matrix3d relative_R;  // 从最新帧到选定帧的旋转（推测的）
        Vector3d relative_T;  // 从最新帧到选定帧的位移（推测的）
        int l;     // 选定帧在滑动窗口中的帧号
        
        // 2. 选择跟最新帧中有足够数量的特征点和视差的某一帧，利用五点法恢复相对旋转和平移量
        // 相对旋转和平移量如果找不到，则初始化失败
        if (!relativePose(0, relative_R, relative_T, l))
        {
            printf("init solve 5pts between first frame and last frame failed\n");
            return false;
        }
        
        // update init progress
        initProgress = 30;
        
        // 三角化：已知两帧的位姿，求解帧中地标点的3D坐标
        /**
         * sfm.construct 函数本身就很巨大，里面很多步骤，大致如下：
         * 1. 三角化l 帧与frame_num - 1帧，得到一些地标点。
         * 2. 通过这些地图点，利用pnp的方法计算l+1, l+2, l+3, …… frame_num-2 相对于l的位姿。
         *           而且每计算一帧，就会与frame_num - 1帧进行三角化，得出更多地标点
         * 3. 三角化l+1, l+2 …… frame_num-2帧与l帧
         * 4. 对l-1, l-2, l-3 等帧与sfm_f的特征点队列进行pnp求解，得出相对于l的位姿，并三角化其与l帧。
         * 5. 三角化剩余的点（某些地标点仅存在与某些帧中，而非在[l,WINDOW]帧中都存在的地标点）
         * 6. ceres全局BA优化，最小化3d投影误差
         */
        
        // 3. 全局SFM初始化滑动窗口中全部初始帧的相机位姿和特征点空间3D位置
        // 3D的feature point和sliding window中的keyFrame的2D feature求解PnP，并且使用ceres优化：
        GlobalSFM sfm;
        if (!sfm.construct(frame_count + 1, Q, T, l,
                          relative_R, relative_T,
                          sfm_f, sfm_tracked_points))
        {
            printf("global SFM failed!");
            init_status = FAIL_SFM;
            marginalization_flag = MARGIN_OLD;
            init_fail_cnt++;
            
            return false;
        }
        
        // update init progress
        initProgress = 50;
    }

    // solve pnp for all frame
    // 因为有continue存在，怎么会solve all，感觉只是不满足时间戳相等的才pnp求解
    //
    // 将相机坐标系转换到IMU坐标系中，然后再一次进行PnP求解，3D特征点还是使用之前SFM中求解出来的，后续也没有进行优化
    //
    // 4. 对于非滑动窗口的所有帧，提供一个初始的R,T，然后solve pnp求解pose
    // 由于并不是第一次视觉初始化就能成功，此时图像帧数目有可能会超过滑动窗口的大小
    // 所以在视觉初始化的最后，需要求出滑动窗口以外的帧的位姿
    // 最后把世界坐标系从帧l的相机坐标系，转到帧l的IMU坐标系
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    
    // TODO: 此处的 i 有点诡异，作用是什么？
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        // all_image_frame与滑动窗口中对应的帧
        if ((frame_it->first) == Headers[i])
        {
            cout << "key frame " << i << endl;
            frame_it->second.is_key_frame = true;  // 滑动窗口中所有帧都是关键帧
            // 根据各帧相机坐标系的姿态和外参，得到用各帧IMU坐标系的姿态
            //（对应VINS Mono论文(2018年的期刊版论文)中的公式（6））
            frame_it->second.R = Q[i].toRotationMatrix() * ric.transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        
        if ((frame_it->first) > Headers[i])
        {
            i++;
        }
        
        // 为滑动窗口外的帧提供一个初始位姿
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        // 罗德里格斯（Rodrigues）旋转向量与矩阵的变换，旋转向量（1x3）与旋转矩阵（3x3）
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);
        
        // 初始化时，位于滑动窗口外的帧是非关键帧
        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;  // 用于pnp解算的3D点
        vector<cv::Point2f> pts_2_vector;  // 用于pnp解算的2D点
        
        // 对于该帧中的特征点
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            //cout << "feature id " << feature_id;
            //cout << " pts image_frame " << (i_p.second.head<2>() * 460 ).transpose() << endl;
            
            // 如果it不是尾部迭代器，说明在sfm_tracked_points中找到了相应的3D点
            it = sfm_tracked_points.find(feature_id);
            if (it != sfm_tracked_points.end())
            {
                // 记录该id特征点的3D位置
                Vector3d world_pts = it->second;
                cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                pts_3_vector.push_back(pts_3);
                
                // 记录该id的特征点在该帧图像中的2D位置
                Vector2d img_pts = id_pts.second.head<2>();
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);
            }
        }
        
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                                               0, 1, 0,
                                               0, 0, 1);
        
        // 如果匹配到的3D点数量少于6个，则认为初始化失败
        if (pts_3_vector.size() < 6)
        {
            printf("init Not enough points for solve pnp !\n");
            return false;
        }
        
        // TODO: 初始化用的frame是不是会越来越多，导致失败的概率增高，是否可以删除一些旧的frame
        // 所有的frame求解PnP（只要有一帧失败就认为fail，是否合理？）
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            printf("init solve pnp fail!\n");
            init_status = FAIL_PNP;
            init_fail_cnt++;
            
            return false;
        }
        
        // pnp求解成功
        // 罗德里格斯（Rodrigues）旋转向量与矩阵的变换，旋转向量（1x3）与旋转矩阵（3x3）
        cv::Rodrigues(rvec, r);
        //cout << "r " << endl << r << endl;
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        //cout << "R_pnp " << endl << R_pnp << endl;
        R_pnp = tmp_R_pnp.transpose();
        
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        
        // 根据各帧相机坐标系的姿态和外参，得到用各帧IMU坐标系的姿态。
        frame_it->second.R = R_pnp * ric.transpose();
        frame_it->second.T = T_pnp;
    }
    
    // update init progress
    initProgress = 75;
    
    printf("init PS after pnp %lf %lf %lf\n", Ps[0].x(), Ps[0].y(), Ps[0].z());
    
    // camera与IMU对齐
    if (visualInitialAlign())
    {
        // update init progress
        initProgress = 85;
        
        return true;
    }
    else
    {
        init_status = FAIL_ALIGN;
        init_fail_cnt++;
        return false;
    }
}

/**
 * cameara与IMU对齐
 */
// Ps:世界坐标下的平移
// Rs:世界坐标下的旋转
// Vs:世界坐标下的速度

// vision IMU数据对齐
// 这里涉及的量有： 陀螺仪的Bias(加速度Bias这里没有处理) 速度V[0:n] 重力g 尺度s
// 更新了Bgs后，则对于imu数据需要repropogate，也就是从当前时刻开始预积分，
//      同时通过三角化和上一步计算的scale可以获得每个feature的深度;
bool VINS::visualInitialAlign()
{
    TS(solve_g);
    VectorXd x;
    // solve scale
    /** 里面包含
     * 1. solveGyroscopeBias()零偏初始化: 在滑动窗口中，每两帧之间的相对旋转
     *    与IMU预积分产生的旋转进行最小二乘法优化，用ldlt求解
     * 2. LinearAlignment() 尺度初始化:由于在视觉初始化SFM的过程中，将其中位姿变化较大的两帧中使用E矩阵
     *    求解出来旋转和位移，后续的PnP和三角化都是在这个尺度下完成的。所以当前的尺度与IMU测量出来的
     *    真实世界尺度肯定不是一致的，所以需要这里进行对齐。这里对齐的方法主要是通过在滑动窗口中每两帧之间
     *    的位置和速度与IMU预积分出来的位置和速度组成一个最小二乘法的形式，然后求解出来
     * 3. RefineGravity()重力向量优化:进一步细化重力加速度，提高估计值的精度，
     *    形式与LinearAlignment()是一致的，只是将g改为g⋅ĝ +w1b1+w2b2
     * 2和3的公式相对难懂，请参考下面图片 https://images2018.cnblogs.com/blog/1072373/201804/1072373-20180419163252748-810017249.png
     */
    /*
     * IMU陀螺仪零偏初始化；主要是通过滑动窗口中，每两帧之间通过SFM求解出来的旋转与
     *         IMU预积分的旋转量组成一个最小二乘法形式的等式，求解出来陀螺仪的零偏。
     * 完成视觉SFM的结果与IMU预积分结果对齐
     */
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if (!result)
    {
        printf("solve g failed!");
        printf("init PS alignment failed %lf %lf %lf\n",
               Ps[0].x(),Ps[0].y(), Ps[0].z());
        return false;
    }
    
    TE(solve_g);
    printf("init PS algnment succ: %lf %lf %lf\n", Ps[0].x(), Ps[0].y(), Ps[0].z());
    
    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        // 滑动窗口中各图像帧在世界坐标系下的旋转和平移
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        // 滑动窗口中所有初始帧都是关键帧
        all_image_frame[Headers[i]].is_key_frame = true;
    }
    
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
    {
        dep[i] = -1;
    }
    f_manager.clearDepth(dep);
    
    // triangulat on cam pose, no tic
    // FeatureManager::triangulate() 三角化
    Vector3d TIC_TMP;
    TIC_TMP.setZero();
    f_manager.triangulate(Ps, TIC_TMP, ric, true);
    
    // 更新相机速度，位置和旋转量(通过精确求解的尺度，重力向量)
    double s = (x.tail<1>())(0);
    
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    
    for (int i = frame_count; i >= 0; i--)
    {
        Ps[i] = s * Ps[i] - Rs[i] * tic - (s * Ps[0] - Rs[0] * tic);
    }
    
    printf("PS after scale %lf %lf %lf\n", Ps[0].x(), Ps[0].y(), Ps[0].z());
    
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if (frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    
    printf("-------------- init visualInitialAlign finish -------------\n");
    
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = (int)it_per_id.feature_per_frame.size();
        
        // 此特征点要至少被2帧图像观测到 && 被第一次观测到的帧不能是倒数2帧以后的帧
        if (!it_per_id.isPriorFeature())
        {
            continue;
        }
        
        it_per_id.estimated_depth *= s;
    }
    
    // 当前参考坐标系与世界坐标系（依靠g构建的坐标系）的旋转矩阵，暂时没搞清楚从谁转到谁？？
    Matrix3d R0 = Utility::g2R(g);
    double yaw0 = Utility::R2ypr(R0).x();
    Matrix3d yaw_refine = Utility::ypr2R(Vector3d{-yaw0, 0, 0});
    R0 = yaw_refine * R0;
    g = R0 * g;
    // Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    
    // 更新相机速度，位置和旋转量(通过精确求解的尺度，重力向量)
    for (int i = 0; i <= frame_count; i++)
    {
        // 似乎是把Ps、Rs、Vs转到世界坐标系下
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
        init_poses.push_back(Ps[i]);
    }
    
    return true;
}

// 判断两帧有足够视差，在滑窗中寻找与最新的关键帧共视关系较强的关键帧
// 在滑动窗口中，寻找与最新帧有足够多数量的特征点对应关系和视差的帧，
// 然后用5点法恢复相对位姿，由对极约束中的F矩阵恢复出相对的R、t
/**
 * relativePose方法中首先通过FeatureManeger获取（滑动窗口中）第i帧和最后一帧的特征匹配corres，
 * 当corres匹配足够大时，考察最新的keyFrame和slidingwindow中某个keyFrame之间有足够
 * feature匹配和足够大的视差（id为l=i），满足这两个条件，然后这两帧之间通过五点法恢复出R，t
 * 并且三角化出3D的特征点feature point，这里是使用solveRelativeRT
 *
 * 这里值得注意的是，这种relativePose得到的位姿是第l帧的，第l帧的筛选是从第一帧开始到
 * 滑动窗口所有帧中一开始满足平均视差足够大的帧，这里的第l帧会作为参考帧到下面的全局SFM使用。
 */
bool VINS::relativePose(int camera_id,
                        Matrix3d &relative_R,
                        Vector3d &relative_T,
                        int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    // 是否找与最新帧共视视差最大的帧？？？
    // 或许应该用 i < WINDOW_SIZE - 2
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        // getCorresponding::WINDOW_SIZE 是最后一帧，也即最新帧
        // 第i帧与最新帧之间的Feature共视
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        // 滑窗内两帧的共视
        // 共视的Features应该大于20
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax += parallax;
            }
            
            // 求取所有匹配的特征点的平均视差
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            parallax_num_view = average_parallax * 520;  // Just for log
            
            // TODO: 此处只计算了第一个共视关系>20的情况，后续可能依然有共视关系>20的情况
            // 此种情况可能不容易初始化成功
//            if (average_parallax * 520 < 30)
            if (average_parallax * 460 < 30)
            {
                init_status = FAIL_PARALLAX;
                init_fail_cnt++;
                return false;  // TODO: 此处或许应该用 continue
//                continue;
            }
            
            // 视差大于一定阈值，并且能够有效地求解出变换矩阵，找到的关键帧设置为l帧
            // solveRelativeRT 利用cv::findFundamentalMat 计算基础矩阵用5点法来得到变换矩阵
            // 利用五点法求解相机初始位姿
            if (m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                printf("average_parallax %f choose l %d and newest frame to triangulate the whole structure\n",
                       average_parallax * 520, l);
                return true;
            }
            else
            {
                init_status = FAIL_RELATIVE;
                init_fail_cnt++;
                return false;  // TODO: 此处或许应该用 continue
//                continue;
            }
        }
    }
    
    return false;
}

/**
 * marginalize the state from the sliding window and change feature start frame
 * 滑动窗口all_image_frame，维持滑动窗口的大小，保证SLAM运行计算的复杂度。
 * 如果第二最新帧是关键帧的话，那么这个关键帧就会留在滑动窗口中，
 * 时间最长的一帧和其测量值就会被边缘化掉；如果第二最新帧不是关键帧的话，
 * 则把这帧的视觉测量舍弃掉而保留IMU测量值在滑动窗口中，这样的策略会保证系统的稀疏性。
 * (0, 1, …, N)关键帧，0是时间最长的关键帧，N是最新关键帧。
 */
// 实际滑动窗口的地方，如果第二最新帧是关键帧的话，那么这个关键帧就会留在滑动窗口中，
// 时间最长的一帧和其测量值就会被边缘化掉如果第二最新帧不是关键帧的话，则把这帧的
// 视觉测量舍弃掉而保留IMU测量值在滑动窗口中这样的策略会保证系统的稀疏性
void VINS::slideWindow()
{
    // marginalize old keyframe
    if (marginalization_flag == MARGIN_OLD)
    {
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);
                
                std::swap(pre_integrations[i], pre_integrations[i + 1]);
                dt_buf[i].swap(dt_buf[i + 1]);
                
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
                
                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                
                // TODO: 此处没有 ba和bg 的swap
//                Bas[i].swap(Bas[i + 1]);
//                Bgs[i].swap(Bgs[i + 1]);
            }
            
            // TODO: 用的是倒数第二个值？？？
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];
            
            if (pre_integrations[WINDOW_SIZE] != NULL)
            {
                delete pre_integrations[WINDOW_SIZE];
            }
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0,
                                                        Bas[WINDOW_SIZE],
                                                        Bgs[WINDOW_SIZE]};
            
            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();
            
            if (solver_flag == INITIAL)
            {
                double t_0 = Headers[0];
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            
            // FeatureManager::removeBack()
            slideWindowOld();
        }
    }
    else  // non keyframe (marginalization_flag == MARGIN_SECOND_NEW)
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];
                
                pre_integrations[frame_count - 1]->push_back(tmp_dt,
                                                    tmp_linear_acceleration,
                                                    tmp_angular_velocity);
                
                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }
            
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];
            
            if (pre_integrations[WINDOW_SIZE]!=NULL)
            {
                delete pre_integrations[WINDOW_SIZE];
            }
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0,
                                                        Bas[WINDOW_SIZE],
                                                        Bgs[WINDOW_SIZE]};
            
            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();
            
            // FeatureManager::removeFront()
            slideWindowNew();
        }
    }
}

/**
 * FeatureManager::removeBack() 将每一个特征点开始出现的帧号减一，如果这个关键点的帧号为0，则直接删除
 */
void VINS::slideWindowOld()
{
//    printf("slideWindowOld -> marginalize back\n");
    
    point_cloud.clear();
    
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = (int)it_per_id.feature_per_frame.size();
        
        // 此特征点要至少被2帧图像观测到 && 被第一次观测到的帧不能是倒数2帧以后的帧
        if (!it_per_id.isPriorFeature())
        {
            continue;
        }
        
        if (it_per_id.start_frame == 0
            && it_per_id.feature_per_frame.size() <= 2
            && it_per_id.solve_flag == SOLVE_SUCC)
        {
            int imu_i = it_per_id.start_frame;
            
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d tmp = Rs[imu_i] * (ric * pts_i + tic) + Ps[imu_i];
            point_cloud.push_back(tmp.cast<float>());
        }
    }
    
    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        
        R0 = back_R0 * ric;
        R1 = Rs[0] * ric;
        P0 = back_P0 + back_R0 * tic;
        P1 = Ps[0] + Rs[0] * tic;
        
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
    {
        // 将每一个特征点开始出现的帧号减一，如果这个关键点的帧号为0，则直接删除
        f_manager.removeBack();
    }
}

void VINS::slideWindowNew()
{
//    printf("----- marginalize front ----- \n");
    f_manager.removeFront(frame_count);
}

