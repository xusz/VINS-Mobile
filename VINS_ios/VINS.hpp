//
//  VINS.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef VINS_hpp
#define VINS_hpp

#include <stdio.h>
#include "feature_manager.hpp"
#include "utility.hpp"
#include "projection_facor.hpp"
#include "pose_local_parameterization.hpp"
#include "global_param.hpp"
#include <ceres/ceres.h>
#include "marginalization_factor.hpp"
#include "imu_factor.h"
#include "draw_result.hpp"
#include <opencv2/core/eigen.hpp>
#include "inital_sfm.hpp"
#include "initial_aligment.hpp"
#include "motion_estimator.hpp"

extern bool LOOP_CLOSURE;

typedef struct RetriveData_t
{
    /* data */
    int old_index;
    int cur_index;
    double header;
    
    Vector3d P_old;
    Quaterniond Q_old;
    Vector3d P_cur;
    Quaterniond Q_cur;
    
    vector<cv::Point2f> measurements;
    vector<int> features_ids;
    bool use;
    
    Vector3d relative_t;
    Quaterniond relative_q;
    double relative_yaw;
    
    double loop_pose[7];
} RetriveData;

// 对应 Estimator
class VINS
{
public:
    
    typedef IMUFactor IMUFactor_t;
    
    VINS();
    
    enum SolverFlag
    {
        INITIAL,     // 还未成功初始化，进行线性初始化
        NON_LINEAR   // 已成功初始化，正处于紧耦合优化状态，进行非线性优化
    };
    
    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,        // marg最老帧
        MARGIN_SECOND_NEW = 1  // marg次新帧
    };
    
    enum InitStatus
    {
        FAIL_IMU,
        FAIL_PARALLAX,
        FAIL_RELATIVE,
        FAIL_SFM,
        FAIL_PNP,
        FAIL_ALIGN,
        FAIL_COST,
        SUCC
    };
    
    /**
     * 后端非线性优化
     */
    void solve_ceres(int buf_num);
    
    void solveCalibration();
    void old2new();
    void new2old();
    
    void setIMUModel();
    void setExtrinsic();
    
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    
    void processImage(map<int, Vector3d> &image_msg,
                      double header,
                      int buf_num);
    
    void processIMU(double t,
                    const Vector3d &linear_acceleration,
                    const Vector3d &angular_velocity);
    
    void clearState();
    void changeState();
    
    /**
     * vins系统初始化
     * 1. 纯视觉初始化
     * 2. imu与视觉对齐
     */
    bool solveInitial();
    
    bool relativePose(int camera_id,
                      Matrix3d &relative_R,
                      Vector3d &relative_T,
                      int &l);
    
    bool visualInitialAlign();
    
    bool failureDetection();
    void failureRecover();
    
    void reInit();
    void update_loop_correction();
    
public:
    FeatureManager  f_manager;    // 用于管理滑动窗口对应的特征点数据
    MotionEstimator m_estimator;
    int frame_count;   // 最新帧在滑动窗口中的索引（0，1，2，... ，WINDOW_SIZE）
    
    // camera与IMU的外参
    Matrix3d ric;   // 从相机到IMU的旋转
    Vector3d tic;   // 从相机到IMU的平移
    
    MarginalizationFlag  marginalization_flag;
    
    // 当前时刻PVQ，此处的计算值 不带noise；两帧frame之间，最多可以存10个IMU数据
    // TODO: 此处的「10*」是否是有用的？
    Vector3d Ps[10 * (WINDOW_SIZE + 1)];   // 滑动窗口中各帧在世界坐标系下的位置
    Vector3d Vs[10 * (WINDOW_SIZE + 1)];   // 滑动窗口中各帧在世界坐标系下的速度
    Matrix3d Rs[10 * (WINDOW_SIZE + 1)];   // 滑动窗口中各帧在世界坐标系下的旋转
    Vector3d Bas[10 * (WINDOW_SIZE + 1)];  // 滑动窗口中各帧对应的加速度计偏置
    Vector3d Bgs[10 * (WINDOW_SIZE + 1)];  // 滑动窗口中各帧对应的陀螺仪偏置
    
    /**
     * 用于ceres优化的参数块，待优化参数
     */
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];  // p_3 & q_4
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS]; // v & ba & bg
    double para_Feature[NUM_OF_F][SIZE_FEATURE];   // λ
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];    // 相机Pose：p_3 & q_4
    
    // for loop closure
    RetriveData retrive_pose_data, front_pose;
    bool loop_enable;
    vector<Vector3f> correct_point_cloud;
    Vector3f correct_Ps[WINDOW_SIZE];
    Matrix3f correct_Rs[WINDOW_SIZE];
    Vector3d t_drift;
    Matrix3d r_drift;
    
    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;
    
    vector<Vector3f> point_cloud;
    
    int feature_num;
    
    // 针对一个滑动窗口的，其中的每个元素保存的都是两帧之间IMU的预积分数据
    // TODO: 此处不应该「10 *」
    IntegrationBase *pre_integrations[10 * (WINDOW_SIZE + 1)];
    bool first_imu;
    // 最近一次接收到的IMU数据，用于下一次IMU积分
    Vector3d acc_0, gyr_0;
    
    /*
     * 滑动窗口中每一帧图像对应的预积分所用到的IMU数据存在3个缓存中
     * 两帧frame之间，最多可以存10个IMU数据
     */
    // IMU数据对应的时间间隔
    vector<double> dt_buf[10 * (WINDOW_SIZE + 1)];
    // 加速度计测量值
    vector<Vector3d> linear_acceleration_buf[10 * (WINDOW_SIZE + 1)];
    // 陀螺仪测量值
    vector<Vector3d> angular_velocity_buf[10 * (WINDOW_SIZE + 1)];
    
    Matrix<double, 7, 1> IMU_linear[10 * (WINDOW_SIZE + 1)];
    Matrix3d IMU_angular[10 * (WINDOW_SIZE + 1)];
    double Headers[10 * (WINDOW_SIZE + 1)];
    
    Vector3d g;
    
    vector<Vector3d> init_poses;
    /*
     * VINS系统完成初始化操作时对应的图像帧的时间戳
     *（需要注意的是，虽然完成了初始化操作，但是初始化不一定成功）
     */
    double initial_timestamp;
    
    Vector3d init_P;
    Vector3d init_V;
    Matrix3d init_R;
    
    SolverFlag solver_flag;
    Matrix3d Rc[10 * (WINDOW_SIZE + 1)];
    
    /*
     * for initialization
     */
    // 存储所有的ImageFrame对象（每读取一帧图像就会构建ImageFrame对象）
    // 键是图像帧的时间戳，值是ImageFrame对象，ImageFrame对象中保存了图像帧的位姿，
    // 相应的预积分和图像特征点信息
    // all_image_frame.size 可能会大于 WINDOW_SIZE
    map<double, ImageFrame> all_image_frame;
    
    // for initialization，用于在创建ImageFrame对象时，
    // 把该指针赋给imageframe.pre_integration
    IntegrationBase *tmp_pre_integration;
    
    Matrix3d back_R0;
    Vector3d back_P0;
    
    // for falure detection
    bool failure_hand;
    bool failure_occur;
    Matrix3d last_R, last_R_old;
    Vector3d last_P, last_P_old;
    
    // for visulization
    DrawResult drawresult;
    cv::Mat image_show;
    cv::Mat imageAI;
    
    InitStatus init_status;
    
    int parallax_num_view;  // Just for log
    int init_fail_cnt;      // Vins初始化失败的次数
    int initProgress;       // 初始化进度(%)
    
    // Just for log
    double final_cost;
    double visual_cost;
    int visual_factor_num;
};

#endif /* VINS_hpp */

