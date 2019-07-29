//
//  feature_tracker.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef feature_tracker_hpp
#define feature_tracker_hpp

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "global_param.hpp"
#include <string>
#include <list>
#include "utility.hpp"
#include <opencv2/core/eigen.hpp>
#include "vins_pnp.hpp"

#define MAX_CNT      70    // 特征点最大个数  原值：150
#define MIN_DIST     30    // 特征点之间的最小间隔
#define COL          480   // 图像宽度
#define ROW          640   // 图像高度
#define F_THRESHOLD  1.0   // ransac算法的门限  ransac threshold (pixel)
/**
 * 光太亮或太暗则为1，进行直方图均衡化
 * if image is too dark or light, trun on equalize to find enough features
 */
#define EQUALIZE    1

using namespace cv;
using namespace std;
using namespace Eigen;

/*
 image frame
 --------> x:480
 |
 |
 |
 |
 |
 | y:640
 */
struct max_min_pts
{
    Point2f min;
    Point2f max;
};

struct IMU_MSG_LOCAL
{
    double header;
    Vector3d acc;
    Vector3d gyr;
};

class FeatureTracker
{
public:
    FeatureTracker();
    
    bool solveVinsPnP(double header, Vector3d &P, Matrix3d &R, bool vins_normal);
    
    void readImage(const cv::Mat &_img,
                   cv::Mat &result,
                   int _frame_cnt,
                   vector<Point2f> &good_pts,
                   vector<double> &track_len,
                   double header,
                   Vector3d &P,
                   Matrix3d &R,
                   bool vins_normal);
    
    void setMask();
    void rejectWithF();
    void addPoints();
    bool updateID(unsigned int i);
    
    /*
     * varialbles
     */
    int frame_cnt;
    cv::Mat mask;  // 图像掩码
    
    /*
     * cur和forw分别是LK光流跟踪的前后两帧
     */
    cv::Mat cur_img;     // cur实际上是上一帧
    cv::Mat forw_img;    // forw才是真正的“当前要处理”的帧
    // 是上一次发布的帧，它实际上是光流跟踪以后，
    // prev和forw根据Fundamental Matrix做RANSAC剔除outlier用的，也就是rejectWithF()函数
    cv::Mat pre_img;
    
    vector<cv::Point2f> n_pts;     // 每一帧中新提取的特征点
    vector<cv::Point2f> cur_pts;   // 对应的图像特征点，正在处理帧的上一帧
    vector<cv::Point2f> pre_pts;
    vector<cv::Point2f> forw_pts;  // 正在处理的帧上的特征点
    
    vector<int> ids;        // 正在处理的帧 能够被跟踪到的特征点的id
    // 代表当前cur_ptrs被追踪的时间次数
    vector<int> track_cnt;  // 当前帧forw_img中每个特征点被追踪的时间次数
    vector<max_min_pts> parallax_cnt;  // 视差量
    /*
     * 用来作为特征点id，每检测到一个新的特征点，就将n_id作为该特征点的id，然后n_id加1
     */
    static int n_id;
    int img_cnt;
    double current_time;
    vinsPnP vins_pnp;
    bool use_pnp;
    
    /*
     * interface
     */
    map<int, Vector3d> image_msg;
    list<IMG_MSG_LOCAL> solved_features;
    VINS_RESULT solved_vins;
    vector<IMU_MSG_LOCAL> imu_msgs;
//    bool update_finished;  // iOS中貌似未用到
};

#endif /* feature_tracker_hpp */

