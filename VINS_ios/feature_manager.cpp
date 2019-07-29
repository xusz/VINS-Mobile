//
//  feature_manager.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/20.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "feature_manager.hpp"

int FeaturePerId::endFrame()
{
    return (int)(start_frame + feature_per_frame.size() - 1);
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
                        : Rs(_Rs)
{
    ric = Utility::ypr2R(Vector3d(RIC_y, RIC_p, RIC_r));
}

double FeatureManager::compensatedParallax1(FeaturePerId &f_per_id)
{
    int l = (int)f_per_id.feature_per_frame.size();
    FeaturePerFrame &frame_i = f_per_id.feature_per_frame[0];
    FeaturePerFrame &frame_j = f_per_id.feature_per_frame[l - 1];
    
    int r_i = f_per_id.start_frame + 0;
    int r_j = f_per_id.start_frame + l - 1;
    
    Vector3d p_i = frame_i.point;
    
    double u_i = p_i(0);
    double v_i = p_i(1);
    
    double ans = 0;
    
    Vector3d p_j = frame_j.point;
    Vector3d p_j_comp;
    p_j_comp = ric.transpose() * Rs[r_i].transpose() * Rs[r_j] * ric * p_j;
    
    double dep_j = p_j(2);
    double u_j = p_j(0) / dep_j;
    double v_j = p_j(1) / dep_j;
    
    double du = u_i - u_j, dv = v_i - v_j;
    double dep_j_comp = p_j_comp(2);
    double u_j_comp = p_j_comp(0) / dep_j_comp;
    double v_j_comp = p_j_comp(1) / dep_j_comp;
    double du_comp = u_i - u_j_comp, dv_comp = v_i - v_j_comp;
    
    double para = sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp));
    
    frame_j.parallax = para;
    
    if (r_i == r_j) //the feature appeared first time
    {
        para = 1e-3;
    }
    ans = max(ans, para);
    
    return ans;
}

/**
 * 对于给定id的特征点
 * 计算第2最新帧和第3最新帧之间该特征点的视差（当前帧frame_count是第1最新帧）
 * （需要使用IMU数据补偿因旋转造成的视差）
 */
double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    // check the second last frame is keyframe or not
    // parallax between second last frame and third last frame
    // 第3最新帧
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    // 第2最新帧
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];
    
    Vector3d p_i = frame_i.point;
    Vector3d p_j = frame_j.point;
    
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;  // 像素平面坐标
    double v_i = p_i(1) / dep_i;  // 像素平面坐标
    
    double dep_j = p_j(2);
    double u_j = p_j(0) / dep_j;  // 像素平面坐标
    double v_j = p_j(1) / dep_j;  // 像素平面坐标
    
    double du = u_i - u_j;
    double dv = v_i - v_j;
    
    double ans = sqrt(du * du + dv * dv);  // 视差补偿值
    return ans;
}

/*
 Check if the current frame has enough parallax compare with previous frame
 if have, return true;
 if no, return false;
 At the sametime, add the new feature observation to the feature class
 */
/**
 * 通过检测两帧之间的视差决定是否作为关键帧，
 * 同时添加之前检测到的特征点到feature（list< FeaturePerId >）容器中，
 * 计算每一个点跟踪的次数，以及它的视差
 */
/**
 * @brief 把图像特征点放入名为feature的list容器中，然后计算当前的视差
 */
/** 选KF策略
 * 把当前帧图像（frame_count）的特征点添加到feature容器中
 * 计算第2最新帧与第3最新帧之间的平均视差（当前帧是第1最新帧）
 * 也就是说当前帧图像特征点存入feature中后，并不会立即判断是否将当前帧添加为新的关键帧，而是去判断当前帧的前一帧（第2最新帧）。
 * 当前帧图像要在下一次接收到图像时进行判断（那个时候，当前帧已经变成了第2最新帧）
 */
// 向Featuresmanger中添加Features并确定共视关系及视差角的大小
/**
 * 1 FeaturePerFrame 特征点在某个图像帧下的坐标（归一化平面坐标）
 * 2 遍历特征点,看该特征点是否在特征点的列表中,如果没在,则将<FeatureID,Start_frame>存入到Feature列表中(如果该Feature之前被观测到过,统计数目)
 * 3 计算共视关系， parallax_num为满足要求的Feature的个数
 *      3.1 至少有两帧观测到该特征点(起始帧要小于倒数第二帧，终止帧要大于倒数第二帧，这样至少有三帧观测到该Feature(包括当前帧))
 *      3.2 判断观测到该特征点的frame中倒数第二帧和倒数第三帧的共视关系 实际是求取该特征点在两帧的归一化平面上的坐标点的距离ans
 *      3.3 得到视差总和并统计个数
 * 4 得到平均视差,平均视差视差要大于某个阈值, MIN_PARALLAX=10，大约是10个像素点.
 *      这样的帧即是关键帧,边缘化最老的帧.否则不是关键帧,边缘化第二最新帧（次新帧）
 *
 * image_msg -- point_clouds，当前帧的点云数据
 */
bool FeatureManager::addFeatureCheckParallax(int frame_count,
                                    const map<int, Vector3d> &image_msg,
                                    int &parallax_num)
{
    double parallax_sum = 0;  // 第2最新帧和第3最新帧之间跟踪到的特征点的总视差
    parallax_num = 0;         // 第2最新帧和第3最新帧之间跟踪到的特征点的数量(满足某些条件的特征点个数)
    last_track_num = 0;       // 当前帧（第1最新帧）图像跟踪到的特征点的数量
    
    // 每个feature有可能出现多个帧中，share same id，放入feature容器中
    // feature容器按照特征点id组织特征点数据，对于每个id的特征点，
    // 记录它被滑动窗口中哪些图像帧观测到了
    for (auto &id_pts : image_msg)
    {
        // 特征点管理器，存储特征点格式：首先按照特征点ID，一个一个存储，每个ID会包含其在不同帧上的位置
        FeaturePerFrame f_per_fra(id_pts.second);
        
        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
                              return it.feature_id == feature_id;
                          });
        // new feature
        // 返回尾部迭代器，说明该特征点第一次出现（在当前帧中新检测的特征点），
        // 需要在feature中新建一个FeaturePerId对象
        if (it == feature.end()) // 如果没有找到此ID，就在管理器中增加此特征点
        {
            // give id and start frame index
            feature.push_back(FeaturePerId(feature_id, frame_count));
            // give point
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        // find match with previous feature
        else if (it->feature_id == feature_id)
        {
            // 如果找到了相同ID特征点，就在其FeaturePerFrame内增加此特征点在此帧的位置以及其他信息，
            // 然后增加last_track_num，说明此帧有多少个相同特征点被跟踪到
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num ++;   // 当前帧（第1最新帧）图像跟踪到的特征点的数量
        }
    }
    
    // 1. 当前帧的帧号小于2，即为0或1，为0，则没有第2最新帧，为1，则第2最新帧是滑动窗口中的第1帧
    // 2. 当前帧（第1最新帧）跟踪到的特征点数量小于20（？？？为什么当前帧的跟踪质量不好，就把第2最新帧当作关键帧？？？）
    // 出现以上2种情况的任意一种，则认为第2最新帧是关键帧
    if (frame_count < 2 || last_track_num < 20)
    {
        return true;   // 第2最新帧是关键帧
    }
    
    // 计算视差，second last和third last
    // 计算第2最新帧和第3最新帧之间跟踪到的特征点的平均视差
    for (auto &it_per_id : feature)
    {
        // 计算能被当前帧和其前两帧共同看到的特征点视差
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            // 对相近的特征点进行视差计算
            // 对于给定id的特征点，计算第2最新帧和第3最新帧之间该特征点的视差（当前帧frame_count是第1最新帧）
            //（需要使用IMU数据补偿由于旋转造成的视差）
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }
    
    printf("parallax sum = %lf parallax_num = %d\n",
           parallax_sum, parallax_num);
    
    if (parallax_num == 0)
    {
        // 如果第2最新帧和第3最新帧之间跟踪到的特征点的数量为0，则把第2最新帧添加为关键帧
        // ？？怎么会出现这种情况？？？？
        // 如果出现这种情况，那么第2最新帧和第3最新帧之间的视觉约束关系不就没有了？？？
        return true;
    }
    else
    {
        //ROS_INFO("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        //ROS_INFO("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        // 计算平均视差
        // 如果平均视差大于设定的阈值，则把第2最新帧当作关键帧
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

/**
 * 得到两帧之间特征点共视关系
 * 第[l,r]帧中都包含某Feature
 */
vector<pair<Vector3d, Vector3d>>
FeatureManager::getCorresponding(int frame_count_l,
                                 int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        // [l,r]帧中都包含此feature
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero();
            Vector3d b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;
            
            a = it.feature_per_frame[idx_l].point;
            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    
    return corres;
}

void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = (int)it_per_id.feature_per_frame.size();
        // 此特征点要至少被2帧图像观测到 && 被第一次观测到的帧不能是倒数2帧以后的帧
        if (!it_per_id.isPriorFeature())
        {
            continue;
        }
        
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

void FeatureManager::triangulate(Vector3d Ps[],
                                 Vector3d tic,
                                 Matrix3d ric,
                                 bool is_nonlinear)
{
    outlier_info.clear();
    
    for (auto &it_per_id : feature)  // 对于每个id的特征点
    {
        if (it_per_id.feature_per_frame.size() >= WINDOW_SIZE)
        {
            it_per_id.fixed = true;
            // cout << "track num" << it_per_id->feature_per_frame.size() << endl;
        }
        
        it_per_id.used_num = (int)it_per_id.feature_per_frame.size();
        // 此特征点要至少被2帧图像观测到 && 被第一次观测到的帧不能是倒数2帧以后的帧
        if (!it_per_id.isPriorFeature())
        {
            continue;
        }
        
        // 该id的特征点深度值大于0说明该点被三角化过，在初始化时为-1
        if (it_per_id.estimated_depth > 0)
        {
            continue;
        }
        
        it_per_id.is_outlier = false;
        
        // 观测到该特征点的第一帧图像在滑动窗口中的帧号
        int imu_i = it_per_id.start_frame;
        // 观测到该特征点的最后一帧图像在滑动窗口中的帧号
        int imu_j = imu_i - 1;
        
        Vector3d p_i = Ps[imu_i], p_j;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point, pts_j;
        
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;
        
        Eigen::Matrix<double, 3, 4> P0;  // 似乎是[R | T]的形式，是一个位姿
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic;
        Eigen::Matrix3d R0 = Rs[imu_i] * ric;
        
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();  // 单位旋转矩阵
        P0.rightCols<1>() = Eigen::Vector3d::Zero();     // 0平移向量
        
        // 对于观测到该id特征点的每一图像帧
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;   // 观测到该特征点的最后一帧图像在滑动窗口中的帧号
            
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic;
            Eigen::Matrix3d R1 = Rs[imu_j] * ric;
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);
            
            // 在第一次进入for循环的时候，这个条件成立，这时候循环体都执行完了，
            // continue发挥不了什么作用啊？？？
            if (imu_i == imu_j)
            {
                continue;
            }
        }
        
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A,
                                    Eigen::ComputeThinV).matrixV().rightCols<1>();
        
        double svd_method = svd_V[2] / svd_V[3];
        
        it_per_id.estimated_depth = svd_method;  // 似乎是得到了该特征点的深度
        
        // 如果估计出来的深度小于0.1（单位是啥？？？），则把它替换为一个设定的值
        if (it_per_id.estimated_depth < 0.1)
        {
            std::vector<int> window_id;
            it_per_id.is_outlier = true;
            it_per_id.estimated_depth = INIT_DEPTH;
            outlier_info.emplace_back(it_per_id.feature_id, window_id);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R,
                                          Eigen::Vector3d marg_P,
                                          Eigen::Matrix3d new_R,
                                          Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        
        if (it->start_frame != 0)
        {
            it->start_frame--;
        }
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                
                if (dep_j > 0)
                {
                    it->estimated_depth = dep_j;
                }
                else
                {
                    it->estimated_depth = INIT_DEPTH;
                }
            }
        }
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == SOLVE_FAIL)
        {
            feature.erase(it);
        }
    }
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = (int)it_per_id.feature_per_frame.size();
        
        // 此特征点要至少被2帧图像观测到 && 被第一次观测到的帧不能是倒数2帧以后的帧
        if (!it_per_id.isPriorFeature())
        {
            continue;
        }
        
        dep_vec(++feature_index) = 1.0 / it_per_id.estimated_depth;
    }
    
    return dep_vec;
}

int FeatureManager::getFeatureCount()
{
    int sum = 0;
    for (auto &it : feature)
    {
        it.used_num = (int)it.feature_per_frame.size();
        
        if (it.used_num >= 2  && it.start_frame < WINDOW_SIZE - 2)
        {
            sum++;
            // if(sum>120)
            //    return ans;
        }
    }
    
    return sum;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = (int) it_per_id.feature_per_frame.size();
        
        // 此特征点要至少被2帧图像观测到 && 被第一次观测到的帧不能是倒数2帧以后的帧
        if (!it_per_id.isPriorFeature())
        {
            continue;
        }
        
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        
        // ROS_INFO("feature id %d , start_frame %d, depth %f ",
        //          it_per_id->feature_id,
        //          it_per_id-> start_frame,
        //          it_per_id->estimated_depth);
        
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = SOLVE_FAIL;
        }
        else
        {
            it_per_id.solve_flag = SOLVE_SUCC;
        }
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

/**
 * 将特征点进行处理，将每一个特征点开始出现的帧号减一，如果这个关键点的帧号为0，则直接删除
 */
void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        
        if (it->start_frame != 0)
        {
            it->start_frame--;
        }
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
            {
                feature.erase(it);
                // printf("remove back\n");
            }
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        
        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            // it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->endFrame() < frame_count - 1)
            {
                continue;
            }
            
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
            {
                feature.erase(it);
                // printf("remove front\n");
            }
        }
        // if(it->is_margin == true) {
        //    feature.erase(it);
        // }
    }
}

