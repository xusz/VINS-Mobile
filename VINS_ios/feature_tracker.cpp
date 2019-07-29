//
//  feature_tracker.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "feature_tracker.hpp"

int FeatureTracker::n_id = 0;

FeatureTracker::FeatureTracker()
              : mask{ROW, COL, CV_8UC1},
//                update_finished{false},
                img_cnt{0},
                current_time{-1.0},
                use_pnp{false}
{
    printf("init ok\n");
}

/******** tools function for feature tracker start ********/
/**
 * @breif 判断特征点是否在指定的范围内
 * 不在指定范围内的点属于边缘点，边缘的特征点会被剔除
 */
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE
        && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

/**
 * @breif 去除无法追踪的特征
 */
template <typename T>
void reduceVector(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
    {
        if (status[i])
        {
            v[j++] = v[i];
        }
    }
    
    v.resize(j);
}

/**
 * 在mask中不为0的区域,调用goodFeaturesToTrack提取新的角点n_pts,
 * 通过addPoints()函数push到forw_pts中, id初始化-1,track_cnt初始化为1.
 */
/**
 * @brief 添加新的特征点
 */
void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
        max_min_pts tmp;
        tmp.min = p;
        tmp.max = p;
        parallax_cnt.push_back(tmp);
    }
}

/**
 * 先对跟踪点forw_pts按跟踪次数降排序, 然后依次选点, 选一个点,
 * 在mask中将该点周围一定半径的区域设为0, 后面不再选取该区域内的点.
 * 有点类似与non-max suppression, 但区别是这里保留track_cnt最高的点
 *
 * setMask() 设置遮挡部分（鱼眼相机）
 * 1. 对检测到的特征点按追踪到的次数排序
 * 2. 在mask图像中将追踪到点的地方设置为0，否则为255，
 *    目的是为了下面做特征点检测的时候可以选择没有特征点的区域进行检测。
 *    在同一区域内，追踪到次数最多的点会被保留，其他的点会被删除
 *
 * 该函数主要用于对跟踪点进行排序并去除密集点。
 * 对跟踪到的特征点，按照被追踪到的次数排序并依次选点；
 * 使用mask进行类似非极大抑制的方法，半径为30，去掉分部密集的点，使特征点分布均匀
 */
/**
 * @brief 对图像使用光流法进行特征点跟踪
 *
 * 按照被追踪到的次数排序，然后加上mask去掉部分点，主要是针对鱼眼相机
 */
void FeatureTracker::setMask()
{
    mask.setTo(255);
    
    // prefer to keep features that are tracked for long time
    
    vector<pair<pair<int, max_min_pts>, pair<cv::Point2f, int>>> cnt_pts_id;
    
    for (unsigned int i = 0; i < forw_pts.size(); i++)
    {
        cnt_pts_id.push_back(make_pair(make_pair(track_cnt[i], parallax_cnt[i]),
                                       make_pair(forw_pts[i], ids[i])));
    }
    
    sort(cnt_pts_id.begin(), cnt_pts_id.end(),
         [](const pair<pair<int, max_min_pts>, pair<cv::Point2f, int>> &a,
            const pair<pair<int, max_min_pts>, pair<cv::Point2f, int>> &b)
         {
             return a.first.first > b.first.first;
         });
    
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();
    parallax_cnt.clear();
    
    for (auto &it : cnt_pts_id)
    {
        // if(true)
        if (mask.at<uchar>(it.second.first) == 255)
        {
            // 当前特征点位置对应的mask值为255，则保留当前特征点，
            // 将对应的特征点位置pts，id，被追踪次数cnt分别存入
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first.first);
            parallax_cnt.push_back(it.first.second);
            
            // 在mask中将当前特征点周围半径为MIN_DIST的区域设置为0，
            // 后面不再选取该区域内的点（使跟踪点不集中在一个区域上）
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
    //for (auto &it: pre_pts)
    //{
    //    cv::circle(mask, it, MIN_DIST, 0, -1);
    //}
}

/**
 * 该函数主要是通过基本矩阵（F）去除外点outliers
 */
/**
 * @breif 通过前后两帧的追踪计算F矩阵，通过F矩阵去除Outliers
 */
void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        vector<uchar> status;
        
        // 调用cv::findFundamentalMat对un_cur_pts和un_forw_pts计算F矩阵
        cv::findFundamentalMat(pre_pts, forw_pts, cv::FM_RANSAC,
                               F_THRESHOLD, 0.99, status);
        reduceVector(pre_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        reduceVector(parallax_cnt, status);
    }
}

/******** tools function for feature tracker ending ********/

/**
 * 求解位姿 P R
 * PnP求解算法是指通过多对3D与2D匹配点，在已知或者未知相机内参的情况下，
 * 利用最小化重投影误差来求解相机外参的算法[后端优化后的数据传给pnp]
 *
 * 寻找第k帧中与滑窗中相同的路标点，计算该路标点在归一化相机系中的坐标，传给feature_msg
 */
bool FeatureTracker::solveVinsPnP(double header, Vector3d &P,
                                  Matrix3d &R, bool vins_normal)
{
    // 未初始化完毕
    if (!vins_normal)
    {
        return false;
    }
    
    /*
     if(solved_features.size() < 2)
     {
     printf("pnp not enough features\n");
     return false;
     }
     */
    vector<IMG_MSG_LOCAL> feature_msg;
    int i = 0;
    
    for (auto &it : solved_features)
    {
        while (ids[i] < it.id)
        {
            i++;
        }
        if (ids[i] == it.id)
        {
            IMG_MSG_LOCAL tmp;
            tmp = it;
            // TODO: what's this  /  归一化到平面？
            tmp.observation = Vector2d((forw_pts[i].x - PX) / FOCUS_LENGTH_X,
                                       (forw_pts[i].y - PY) / FOCUS_LENGTH_Y);
            feature_msg.push_back(tmp);
        }
    }
    /*
     if(feature_msg.size() < 2 )
     {
     printf("pnp Not enough solved feature!\n");
     return false;
     }
     */
    // 对PnP的小滑窗（比如6个相邻帧）内的PVQB进行赋值，
    // 当小滑窗内有第i帧时，则用从后端传过来的第i帧PVQ来更新
    vins_pnp.setInit(solved_vins);
    
    printf("pnp imu header: ");
    
    for (auto &it : imu_msgs)
    {
        double t = it.header;
        if (current_time < 0)
        {
            current_time = t;
        }
        
        double dt = (t - current_time);
        current_time = t;
        printf("%lf ", t);
        // 对第k-1帧到第k帧预积分
        vins_pnp.processIMU(dt, it.acc, it.gyr);
    }
    
    printf("image %lf \n", header);
    
    // 进行小滑窗优化
    // 首先通过updateFeatures更新小滑窗中与第k帧有共视关系的帧的3D点坐标P_w，
    // 最后调用solve_ceres来优化小滑窗内的PVQ，不优化路标点，
    // 涉及的两个约束为：IMU的帧间约束，和每一帧的PnP（即每一帧的2D-3D观测）的视觉重投影误差约束
    vins_pnp.processImage(feature_msg, header, use_pnp);
    
    P = vins_pnp.Ps[PNP_SIZE - 1];
    R = vins_pnp.Rs[PNP_SIZE - 1];
//    Vector3d R_ypr = Utility::R2ypr(R);
    
    return true;
}

/**
 *  readImage()的处理流程为:
 
 ①先调用cv::CLAHE对图像做直方图均衡化(如果EQUALIZE=1，表示太亮或则太暗)
 
 PS：CLAHE是一种直方图均衡算法，能有效的增强或改善图像（局部）对比度，从而获取更多图像相关边缘信息有利于分割，比如在书架识别系统的书脊切割中，使用CLAHE可以比传统的直方图增强方法达到更好的增强书脊边界直线的效果，从而有利于后续的书脊边界直线的检测和提取。还能够有效改善AHE中放大噪声的问题，虽然在实际中应用不多，但是效果确实不错。
 
 ②调用calcOpticalFlowPyrLK()跟踪cur_pts到forw_pts,根据status,把跟踪失败的点剔除(注意:prev, cur,forw, ids, track_cnt都要剔除),这里还加了个inBorder判断,把跟踪到图像边缘的点也剔除掉.
 
 ③如果不需要发布特征点,则到这步就完了,把当前帧forw赋给上一帧cur, 然后退出.如果需要发布特征点(PUB_THIS_FRAME=1), 则执行下面的步骤
 
 ④先调用rejectWithF()对prev_pts和forw_pts做ransac剔除outlier.(实际就是调用了findFundamentalMat函数), 在光流追踪成功就记被追踪+1，数值代表被追踪的次数，数值越大，说明被追踪的就越久
 
 ⑤调用setMask(), 先对跟踪点forw_pts按跟踪次数降排序, 然后依次选点, 选一个点, 在mask中将该点周围一定半径的区域设为0, 后面不再选取该区域内的点. 有点类似与non-max suppression, 但区别是这里保留track_cnt最高的点.
 
 ⑥在mask中不为0的区域,调用goodFeaturesToTrack提取新的角点n_pts, 通过addPoints()函数push到forw_pts中, id初始化-1,track_cnt初始化为1.
 
 整体来说需要注意的是：光流跟踪在②中完成，角点提取在⑥中完成
 */
void FeatureTracker::readImage(const cv::Mat &_img,
                               cv::Mat &result,
                               int _frame_cnt,
                               vector<Point2f> &good_pts,
                               vector<double> &track_len,
                               double header,
                               Vector3d &P,
                               Matrix3d &R,
                               bool vins_normal)
{
    result = _img;
    if (forw_img.empty())
    {
        pre_img = cur_img = forw_img = _img;
    }
    else
    {
        forw_img = _img;  // forw_img是名副其实的当前「要处理」的帧
    }
    
    forw_pts.clear();  // 此时forw_pts还保存的是上一帧图像中的特征点，所以把它清除
    
    // track
    {
        if (cur_pts.size() > 0)
        {
            vector<uchar> status;
            vector<float> err;
            
            //TS(time_track);
            /*
             * 跟踪cur_pts到forw_pts，根据status把跟踪失败的点剔除
             * (注意：prev, cur, forw, ids, track_cnt都要剔除)，
             * 这里还加了个inBorder判断，把跟踪到图像边缘的点也剔除掉
             */
            /**
             * 调用cv::calcOpticalFlowPyrLK() 对前一帧的特征点cur_pts
             * 进行LK金字塔光流跟踪，得到forw_pts; status标记了从前一帧cur_img
             * 到forw_img特征点的跟踪状态，无法被追踪到的点标记为0
             */
            calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts,
                                 status, err, cv::Size(21, 21), 3);
            // TE(time_track);
            // 把跟踪到图像边缘的点也剔除掉
            int ptCount = int(forw_pts.size());
            for (int i = 0; i < ptCount; i++)
            {
                if (status[i] && !inBorder(forw_pts[i]))
                {
                    status[i] = 0;
                }
            }
            
            // 缩减Vector，只保留status=1的数据，把跟踪失败的点&边缘点剔除
            // 不仅要从当前帧forw_pts中剔除，还要从cur_un_pts、prev_pts和cur_pts中剔除
            // 记录特征点id的ids，和记录特征点被跟踪次数的track_cnt也要剔除
            reduceVector(pre_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(forw_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(parallax_cnt, status);
            
            // reject outliers，剔除离群点，消除cur_pts和forw_pts的误匹配
            if (forw_pts.size() >= 8)
            {
                vector<uchar> status;
                
                // 对于每对匹配点，基于校正后的位置，用F矩阵加ransac来筛选
                // RANSAC消除误匹配点，status用于存储RANSAC后每个点的状态，是否是野点
                cv::findFundamentalMat(cur_pts, forw_pts, cv::FM_RANSAC,
                                       F_THRESHOLD, 0.99, status);
                reduceVector(cur_pts, status);
                reduceVector(pre_pts, status);
                reduceVector(forw_pts, status);
                reduceVector(ids, status);
                reduceVector(track_cnt, status);
                reduceVector(parallax_cnt, status);
            }
            
            // TODO: 如何求解位姿
//            solveVinsPnP(header, P, R, vins_normal);
            
            if (img_cnt != 0)
            {
                for (int i = 0; i < forw_pts.size(); i++)
                {
                    //cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
                    good_pts.push_back(forw_pts[i]);
                    if (forw_pts[i].x < parallax_cnt[i].min.x
                        || forw_pts[i].y < parallax_cnt[i].min.y)
                    {
                        parallax_cnt[i].min = forw_pts[i];
                    }
                    else if (forw_pts[i].x > parallax_cnt[i].max.x
                            || forw_pts[i].y > parallax_cnt[i].max.y)
                    {
                        parallax_cnt[i].max = forw_pts[i];
                    }
                    // 视差归一化？
                    double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0 ? 0 : cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                    track_len.push_back(std::min(1.0, 1.0 * parallax/30));
                }
            }
        }
    }  // track
    
    // detect
    {
        /*
         * 先调用rejectWithF()对prev_pts和forw_pts做ransac剔除outlier
         * (实际就是调用了findFundamentalMat函数),
         * 在光流追踪成功就记被追踪+1，数值代表被追踪的次数，数值越大，说明被追踪的就越久
         */
        if (img_cnt == 0)
        {
            // 通过F矩阵去除外点
            rejectWithF();
            
            for (int i = 0; i< forw_pts.size(); i++)
            {
                good_pts.push_back(forw_pts[i]);
                if(forw_pts[i].x < parallax_cnt[i].min.x
                   || forw_pts[i].y < parallax_cnt[i].min.y)
                {
                    parallax_cnt[i].min = forw_pts[i];
                }
                else if(forw_pts[i].x > parallax_cnt[i].max.x
                        || forw_pts[i].y > parallax_cnt[i].max.y)
                {
                    parallax_cnt[i].max = forw_pts[i];
                }
                double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0 ? 0 : cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                track_len.push_back(std::min(1.0, 1.0 * parallax/50));
            }
            
            // 光流追踪成功,特征点被成功跟踪的次数就加1
            // 数值代表被追踪的次数，数值越大，说明被追踪的就越久
            for (auto &n : track_cnt)
            {
                n++;
            }
            
            /*
             * 保证相邻的特征点之间要相隔30个像素,设置mask
             * setMask() 设置遮挡部分（鱼眼相机）
             * 1. 对检测到的特征点按追踪到的次数排序
             * 2. 在mask图像中将追踪到点的地方设置为0，否则为255，
             * 目的是为了下面做特征点检测的时候可以选择没有特征点的区域进行检测。
             * 在同一区域内，追踪到次数最多的点会被保留，其他的点会被删除
             */
            // 为下面的goodFeaturesToTrack保证相邻的特征点之间要相隔30个像素,设置mask image
            setMask();
            
            // if 特征点<MAX_CNT，则额外提取新的角点
            int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
            if (n_max_cnt > 0)
            {
                n_pts.clear();
                TS(time_goodfeature);
                // goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.10, MIN_DIST, mask, 3, false, 0.04);
                /*
                 * 在mask中不为0的区域,调用goodFeaturesToTrack提取新的角点n_pts,
                 * 角点提取在此处完成
                 */
                /**
                 * cv::goodFeaturesToTrack()
                 * @brief   在mask中不为0的区域检测新的特征点
                 * @param[in]    InputArray _image=forw_img 输入图像
                 * @param[out]   _corners=n_pts 存放检测到的角点的vector
                 * @param[in]    maxCorners=MAX_CNT - forw_pts.size() 返回的角点的数量的最大值
                 * @param[in]    qualityLevel=0.01 角点质量水平的最低阈值（范围为0到1，质量最高角点的水平为1），小于该阈值的角点被拒绝
                 * @param[in]    minDistance=MIN_DIST 返回角点之间欧式距离的最小值
                 * @param[in]    _mask=mask 和输入图像具有相同大小，类型必须为CV_8UC1,用来描述图像中感兴趣的区域，只在感兴趣区域中检测角点
                 * @param[in]    blockSize：计算协方差矩阵时的窗口大小
                 * @param[in]    useHarrisDetector：指示是否使用Harris角点检测，如不指定，则计算shi-tomasi角点
                 * @param[in]    harrisK：Harris角点检测需要的k值
                 * @return       void
                 */
                // 上面通过光流法找到一些对应点，这里是为了确保每个帧有足够点，
                // 然后调用addPoint添加点
                goodFeaturesToTrack(forw_img, n_pts, n_max_cnt,
                                    0.01, MIN_DIST, mask);
                TE(time_goodfeature);
            }
            else
            {
                n_pts.clear();
            }
            
            /*
             * 添加新检测到的特征点
             * 提取到的新角点通过addPoints()函数push到forw_pts中，
             * id初始化-1，track_cnt初始化为1
             */
            addPoints();
            //printf("features num after detect: %d\n",static_cast<int>(forw_pts.size()));
            pre_img = forw_img;
            pre_pts = forw_pts;
            
            // draw
            for (int i = 0; i < n_pts.size(); i++)
            {
                good_pts.push_back(n_pts[i]);
                track_len.push_back(0);
            }
            // result = mask;
        }
        
        cur_img = forw_img;
        cur_pts = forw_pts;
    } // detect
    
    if (img_cnt == 0)
    {
        //update id and msg
        image_msg.clear();
        
        for (unsigned int i = 0; ; i++)
        {
            bool completed = false;
            completed |= updateID(i);
            
            if (!completed)
            {
                break;
            }
        }
        
        // 归一化? 将所有特征点转换到一个归一化平面并且进行畸变???
        for (int i = 0; i < ids.size(); i++)
        {
            double x = (cur_pts[i].x - PX) / FOCUS_LENGTH_X;
            double y = (cur_pts[i].y - PY) / FOCUS_LENGTH_Y;
            double z = 1.0;
            image_msg[(ids[i])] = (Vector3d(x, y, z));
        }
    }
    
    // finished and tell solver the data is ok
//    update_finished = true;   // iOS中貌似未用到
}


bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
        {
            ids[i] = n_id++;
        }
        return true;
    }
    else
    {
        return false;
    }
}

