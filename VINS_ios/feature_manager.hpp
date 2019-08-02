//
//  feature_manager.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/20.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef feature_manager_hpp
#define feature_manager_hpp

#include <stdio.h>
#include <list>
#include <vector>
#include <stdlib.h>
#include <iostream>
#include "feature_tracker.hpp"
#include <eigen3/Eigen/Dense>
#include "global_param.hpp"
#include "utility.hpp"

#define COMPENSATE_ROTATION  false
#define MIN_PARALLAX_POINT  ((double)(3.0/549))
#define MIN_PARALLAX        ((double)(10.0/549))
#define INIT_DEPTH          ((double)(5.0))

using namespace Eigen;
using namespace std;
using namespace std;


enum SOLVE_FLAG {
    SOLVE_NONE = 0,  // haven't solve yet
    SOLVE_SUCC,      // solve succ
    SOLVE_FAIL       // solve fail;
};

// 特征点在某个图像帧下的坐标（归一化平面坐标）
class FeaturePerFrame
{
public:
    FeaturePerFrame(const Vector3d &_point)
    {
        z = _point(2);
        point = _point / z;
    }
    
    Vector3d point;
    double z;
    bool is_used;
    double parallax;
    double dep_gradient;
};

class FeaturePerId
{
public:
    const int feature_id;
    int start_frame;
    
    // feature_id 在哪些帧中被观察到
    vector<FeaturePerFrame> feature_per_frame;
    
    int used_num;     // 此特征点被多少frame观测到
    bool is_margin;
    bool is_outlier;
    
    double estimated_depth;
    
    bool fixed;
    bool in_point_cloud;
    
    // SOLVE_FLAG : 0 haven't solve yet; 1 solve succ; 2 solve fail;
    int solve_flag;
    
    FeaturePerId(int _feature_id, int _start_frame)
                : feature_id(_feature_id),
                  start_frame(_start_frame),
                  used_num(0),
                  estimated_depth(-1.0),
                  is_outlier(false),
                  fixed(false),
                  in_point_cloud(false)
    {
    }
    
    int endFrame();
    
    // 是否可作为先验特征，被观测到的次数>=2次 && 起始帧不在最后两帧
    inline bool isPriorFeature()
    {
        return (used_num >= 2 && start_frame < WINDOW_SIZE - 2);
    }
};

class FeatureManager
{
public:
    FeatureManager(Matrix3d _Rs[]);
    
    bool addFeatureCheckParallax(int frame_count,
                                 const map<int,Vector3d> &image_msg,
                                 int &parallax_num);
    
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l,
                                                      int frame_count_r);
    
    void triangulate(Vector3d Ps[],
                     Vector3d tic,
                     Matrix3d ric,
                     bool is_nonlinear);
    
    VectorXd getDepthVector();
    
    int getFeatureCount();
    void clearState();
    void tagMarginalizedPoints(bool marginalization_flag);
    void removeBack();
    void removeFront(int frame_count);
    void setDepth(const VectorXd &x);
    void clearDepth(const VectorXd &x);
    void shift(int n_start_frame);
    void removeFailures();
    
    void removeBackShiftDepth(Eigen::Matrix3d marg_R,
                              Eigen::Vector3d marg_P,
                              Eigen::Matrix3d new_R,
                              Eigen::Vector3d new_P);
    
    /*
     * variables
     */
    list<FeaturePerId> feature;   // 管理滑动窗口中所有的特征点
    std::vector<std::pair<int, std::vector<int>>> outlier_info;
    int last_track_num;           // 最新帧图像跟踪到的特征点的数量
    
private:
//    double compensatedParallax1(FeaturePerId &it_per_id);
    
    double compensatedParallax2(const FeaturePerId &it_per_id,
                                int frame_count);
    
    const Matrix3d *Rs;
    Matrix3d ric;
};

#endif /* feature_manager_hpp */

