#include "initial_sfm.hpp"
#include "global_param.hpp"

GlobalSFM::GlobalSFM(){}

/**
 * 三角化恢复特征点3D位置
 *
 * 参考 https://blog.csdn.net/u012101603/article/details/79714332
 */
void GlobalSFM::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0,
                                 Eigen::Matrix<double, 3, 4> &Pose1,
                                 Vector2d &point0,
                                 Vector2d &point1,
                                 Vector3d &point_3d)
{
    Matrix4d design_matrix = Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    
    Vector4d triangulated_point =
        design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

/**
 * 对l+1, l+2, l+3, … frame_num-2 帧与sfm_f特征点队列进行PnP求解
 * 得到这些帧的相机在空间的位置，三角化 l, l+1， l+2, … frame_num-2 帧与frame_num-1帧，
 * 得到这些特征点在空间的3D位置
 */
bool GlobalSFM::solveFrameByPnP(Matrix3d &R_initial,
                                Vector3d &P_initial,
                                int i,
                                vector<SFMFeature> &sfm_f)
{
    vector<cv::Point2f> pts_2_vector;   // 用于pnp解算的3D点
    vector<cv::Point3f> pts_3_vector;   // 用于pnp解算的2D点
    
    for (int j = 0; j < feature_num; j++)  // 遍历sfm_f中的特征点
    {
        // 如果该特征点未被三角化，则跳过该特征点
        if (sfm_f[j].state != true) {
            continue;
        }
        
        Vector2d point2d;
        // 遍历观测到该特征点的图像帧
        for (int k = 0; k < (int)sfm_f[j].observation.size(); k++)
        {
            // 从observation中找到待解算位姿的帧i
            if (sfm_f[j].observation[k].first == i)
            {
                // 获取用于pnp解算的2D坐标
                Vector2d img_pts = sfm_f[j].observation[k].second;
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);
                
                // 获取用于pnp解算的3D坐标
                cv::Point3f pts_3(sfm_f[j].position[0],
                                  sfm_f[j].position[1],
                                  sfm_f[j].position[2]);
                pts_3_vector.push_back(pts_3);
                break;
            }
        }
    }
    
    // 如果内点数量小于15，则表明初始化过程中特征点跟踪不稳定
    if (int(pts_2_vector.size()) < 15)
    {
        cout << "feature tracking not enough, please slowly move you device!" << endl;
        return false;
    }
    
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    // 似乎是因为2D点使用了归一化平面上的坐标，所以相机内参矩阵设置为单位阵
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1);
    
    bool pnp_succ = cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1);
    if (!pnp_succ)
    {
        cout << "solveFrameByPnP::pnp failed !" << endl;
        return false;
    }
    
    cv::Rodrigues(rvec, r);  // 旋转向量转为旋转矩阵

    MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    
    R_initial = R_pnp;   // 输出旋转矩阵
    P_initial = T_pnp;   // 输出平移向量

    return true;
}

/**
 * 三角化两帧内的所有对应feature，放入sfm_f中，三角化成功的state设为true
 */
void GlobalSFM::triangulateTwoFrames(int frame0,
                                     Eigen::Matrix<double, 3, 4> &Pose0,
                                     int frame1,
                                     Eigen::Matrix<double, 3, 4> &Pose1,
                                     vector<SFMFeature> &sfm_f)
{
    assert(frame0 != frame1);
    for (int j = 0; j < feature_num; j++)  // 遍历sfm_f中的特征点
    {
        // 该特征点已经被三角化，则跳过
        if (sfm_f[j].state == true) {
            continue;
        }
        bool has_0 = false;
        bool has_1 = false;
        Vector2d point0;  // frame0中的2D点
        Vector2d point1;  // frame1中的2D点
        
        // 遍历观测到该特征点的图像帧
        for (int k = 0; k < (int)sfm_f[j].observation.size(); k++)
        {
            if (sfm_f[j].observation[k].first == frame0)
            {
                // frame0 观测到了该特征点，记录数据
                point0 = sfm_f[j].observation[k].second;
                has_0 = true;
            }
            if (sfm_f[j].observation[k].first == frame1)
            {
                // frame1 观测到了该特征点，记录数据
                point1 = sfm_f[j].observation[k].second;
                has_1 = true;
            }
        }
        
        // frame0和frame1同时观测到了该特征点，则进行三角化
        if (has_0 && has_1)
        {
            Vector3d point_3d;
            triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
            sfm_f[j].state = true;  // 该特征点被成功三角化
            sfm_f[j].position[0] = point_3d(0);
            sfm_f[j].position[1] = point_3d(1);
            sfm_f[j].position[2] = point_3d(2);
            //cout << "trangulated : " << frame1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
        }
    }
}

// q w_R_cam t w_R_cam
// c_rotation cam_R_w
// c_translation cam_R_w
// relative_q[i][j]  j_q_i
// relative_t[i][j]  j_t_ji  (j < i)
/**
 * 全局SFM初始化全部初始帧中的相机位置和特征点空间3D位置
 *（l是F矩阵初始化得到的，下面做的SFM，就是将l帧作为系统初始化原点，
 * 通过PnP和三角化得到滑动窗口中l帧之后和l帧之前的相机位置和3D特征点）
 */
/**
  4.4.2.4.1 将第l帧当做原点，建立坐标系,然后将Frame转换到同一坐标系下。
  4.4.2.4.2 获得最后一帧到l帧的变换关系
  4.4.2.4.3 获得世界坐标系到l帧的变换关系
  4.4.2.4.4 求取世界坐标系到最后一帧到的变换关系
  4.4.2.4.5 先第l帧和最后一帧做三角化,根据三角化的点PnP求解第l+1的位姿,再l+1和最后一帧三角化,再PnP求l+2位姿,依次求到倒数第二帧的位姿
  4.4.2.4.6 第l+1帧~倒数第二帧的所有帧与第l帧三角化剩下的特征点
  4.4.2.4.7 从l帧开始往第一帧，逐渐帧pnp，再与第l帧进行三角定位得到更多的三维点
  4.4.2.4.8 剩下的那些还没有被三角定位的特征点，通过它被观察到的第一帧和最后一帧进行三角定位。
  4.4.2.4.9 BA优化，使用的是ceres优化位姿和特征点，residuals用的特征点到每一帧重投影误差, 将求解结果写出到q,T和sfm_tracked_points
 */
/**
 * \brief 外部調用接口,主要處理函數. 輸入第l幀和最後一幀的相對R,t, 根據特徵點的觀測估計所有幀的位姿和特徵點的3D座標
 * 输入参数说明：
 * int frame_num： 滑动窗口中图像帧的数量，实际上就是WINDOW_SIZE + 1 / pose的個數, elements in q,T
 * Quaterniond* q： 一个数组指针，数组中需要存储滑动窗口中所有帧的姿态/SFM結果,每幀在l幀參考系下的quaternion
 * Vector3d* T： 一个数组指针，数组中需要存储滑动窗口中所有帧的位置/SFM結果,每幀在l幀參考系下的position
 * int l： 在滑动窗口中找到的与最新帧做5点法的帧的帧号/以第l幀爲參考系,即l幀的pose爲座標原點
 * const Matrix3d relative_R： 第l幀到最後一幀的相對旋轉
 * const Vector3d relative_T： 第l幀到最後一幀的相對平移
 * vector<SFMFeature> &sfm_f： 用于视觉初始化的特征点数据,每個SFMFeature中包含多個觀測
 * map<int, Vector3d> &sfm_tracked_points： 優化後的3D特徵點在l幀參考系的position
 */
bool GlobalSFM::construct(int frame_num,
                          Quaterniond* q,
                          Vector3d* T,
                          int l,
                          const Matrix3d relative_R,
                          const Vector3d relative_T,
                          vector<SFMFeature> &sfm_f,
                          map<int, Vector3d> &sfm_tracked_points)
{
    feature_num = (int)sfm_f.size();

    // intial two view
    // 把relativePose找到的第l帧作为初始位置，最后一帧的pose为relative_R,relative_T
    // （是字母l，不是数字1，l是在滑动窗口中找到的与最新帧做5点法的帧的帧号）
    // 第l帧的姿态设置为一个没有任何旋转的实单位四元数，第l帧作为初始状态帧
    q[l].w() = 1;
    q[l].x() = 0;
    q[l].y() = 0;
    q[l].z() = 0;
    
    // 第l帧位置向量设置为[0, 0, 0]
    T[l].setZero();
    
    // 第l帧（是字母l，不是数字1）到滑动窗口中最新帧的旋转和位移
    q[frame_num - 1] = q[l] * Quaterniond(relative_R);
    T[frame_num - 1] = relative_T;
    //cout << "init q_l " << q[l].w() << " " << q[l].vec().transpose() << endl;
    //cout << "init t_l " << T[l].transpose() << endl;
    
    // rotate to cam frame
	Matrix3d c_Rotation[frame_num];
	Vector3d c_Translation[frame_num];
	Quaterniond c_Quat[frame_num];
    // for ceres
    double c_rotation[frame_num][4];
    double c_translation[frame_num][3];
    
    // 滑动窗口中各帧在世界坐标系（一开始是把第l帧相机坐标系作为世界坐标系起始点）中的位姿
	Eigen::Matrix<double, 3, 4> Pose[frame_num];  // [R|t]
    
    /*
     * 第l帧
     */
    // 四元数取逆（实际上就是共轭），相当于旋转矩阵取逆，
    // 得到从第l帧相机坐标系到第l帧相机坐标系的旋转
    c_Quat[l] = q[l].inverse();
    c_Rotation[l] = c_Quat[l].toRotationMatrix();  // 四元数转旋转矩阵
    // 从第l帧相机坐标系到第l帧相机坐标系的位移
    c_Translation[l] = -1 * (c_Rotation[l] * T[l]);
    Pose[l].block<3, 3>(0, 0) = c_Rotation[l];
    Pose[l].block<3, 1>(0, 3) = c_Translation[l];
    
    /*
     * 滑动窗口中的最新帧
     */
    // 从第l帧相机坐标系到最新帧相机坐标系的旋转
    c_Quat[frame_num - 1] = q[frame_num - 1].inverse();
    c_Rotation[frame_num - 1] = c_Quat[frame_num - 1].toRotationMatrix();
    // 从第l帧相机坐标系到最新帧相机坐标系的位移
    c_Translation[frame_num - 1] = -1 * (c_Rotation[frame_num - 1] * T[frame_num - 1]);
    Pose[frame_num - 1].block<3, 3>(0, 0) = c_Rotation[frame_num - 1];
    Pose[frame_num - 1].block<3, 1>(0, 3) = c_Translation[frame_num - 1];
    
    // 1: trangulate between l ----- frame_num - 1
    // 2: solve pnp l + 1; trangulate l + 1 ------- frame_num - 1;
    //   ...   solve pnp l + 1; trigangulate 0 ----- l;
    // 1: trangulate between l ----- frame_num - 1
    // 2: solve pnp l + 1; trangulate l + 1 ------- frame_num - 1;
    // 以frame_num - 1为参考帧，根据第l和frame_num - 1帧的R,T，三角化一些点，
    //            然后再用PNP得到l+1到frame-1之间所有相对pose，然后恢复这些3D点
    /**
     * 根据第l和frame_num - 1帧的R,T，三角化一些点
     *     （此时的世界坐标系，即参考坐标系，推断应该是第l帧的相机坐标系）
     * 从第l帧到第（frame_num - 2）帧：
     *      先通过pnp计算第i帧（i = l + 1, ..., frame_num - 2）的位姿（第l帧的位姿已经得到，不再计算）
     *      再调用triangulateTwoFrames()，与第frame_num - 1帧进行匹配，三角化一些点
     */
    for (int i = l; i < frame_num - 1 ; i++)
    {
        // solve pnp, 求解位姿
        if (i > l)
        {
            Matrix3d R_initial = c_Rotation[i - 1];
            Vector3d P_initial = c_Translation[i - 1];
            
            if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
            {
                return false;
            }
            
            c_Rotation[i] = R_initial;
            c_Translation[i] = P_initial;
            c_Quat[i] = c_Rotation[i];
            Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
            Pose[i].block<3, 1>(0, 3) = c_Translation[i];
        }
        
        // triangulate point based on the solve pnp result
        // 遍历l帧到第(frame_num-2)帧，寻找与第(frame_num-1)帧的匹配，三角化特征点
        triangulateTwoFrames(i, Pose[i], frame_num - 1,
                             Pose[frame_num - 1], sfm_f);
    }
    
    // 3: triangulate l-----l+1 l+2 ... frame_num -2
    // 三角化l+1, l+2 … frame_num-2帧与l帧
    // 3: triangulate l-----l+1 l+2 ... frame_num -2
    // 以l为参考帧，继续恢复3D点
    // 遍历（l + 1）帧到第（frame_num - 2）帧，寻找与第l帧的匹配，三角化更多的地图点
    /**
     * 从第l + 1帧到第（frame_num - 2）帧：
     * 第i帧（i = l + 1, ..., frame_num - 2）的位姿前面已经计算过了
     * 再次调用triangulateTwoFrames()，与第l帧进行匹配，再三角化一些特征点
     */
    for (int i = l + 1; i < frame_num - 1; i++)
    {
        triangulateTwoFrames(l, Pose[l], i, Pose[i], sfm_f);
    }
    
    // 4: solve pnp l-1; triangulate l-1 ----- l
    //              l-2              l-2 ----- l
    //
    // 对l-1, l-2, l-3, …, 0帧与sfm_f特征点队列进行PnP求解，
    // 得到这些帧的相机在空间的位置,三角化l-1, l-2, l-3, …, 0帧与l帧
    // 以l为参考帧，恢复0-l的pose和3D点
    /**
     * 从第l - 1帧到第0帧：
     * 先通过pnp计算第i帧（i = l - 1, ..., 0）的位姿
     * 再调用triangulateTwoFrames()，与第l帧进行匹配，三角化一些特征点
     */
    for (int i = l - 1; i >= 0; i--)
    {
        // solve pnp
        Matrix3d R_initial = c_Rotation[i + 1];
        Vector3d P_initial = c_Translation[i + 1];
        
		if(!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
		{
			return false;
		}

        c_Rotation[i] = R_initial;
        c_Translation[i] = P_initial;
        c_Quat[i] = c_Rotation[i];
        Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
        Pose[i].block<3, 1>(0, 3) = c_Translation[i];
        
        // triangulate
        triangulateTwoFrames(i, Pose[i], l, Pose[l], sfm_f);
    }
    
    // 5: triangulate all other points
    // 根据以上的点，恢复其他的点
    /**
     * 对于sfm_f中没有被三角化的点，进行三角化
     */
    for (int j = 0; j < feature_num; j++)
    {
        // 该特征点已经三角化，跳过
        if (sfm_f[j].state == true) {
            continue;
        }
        
        if ((int)sfm_f[j].observation.size() >= 2)
        {
            int frame_0 = sfm_f[j].observation[0].first;
            int frame_1 = sfm_f[j].observation.back().first;
            Vector2d point0 = sfm_f[j].observation[0].second;
            Vector2d point1 = sfm_f[j].observation.back().second;
            Vector3d point_3d;
            
            triangulatePoint(Pose[frame_0],
                             Pose[frame_1],
                             point0,
                             point1,
                             point_3d);
            
            sfm_f[j].state = true;
            sfm_f[j].position[0] = point_3d(0);
            sfm_f[j].position[1] = point_3d(1);
            sfm_f[j].position[2] = point_3d(2);
            //cout << "trangulated : " << frame_0 << " " << frame_1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
        }
    }
    
    /*
     for (int i = 0; i < frame_num; i++)
     {
     q[i] = c_Rotation[i].transpose();
     cout << "solvePnP  q" << " i " << i <<"  " <<q[i].w() << "  " << q[i].vec().transpose() << endl;
     }
     for (int i = 0; i < frame_num; i++)
     {
     Vector3d t_tmp;
     t_tmp = -1 * (q[i] * c_Translation[i]);
     cout << "solvePnP  t" << " i " << i <<"  " << t_tmp.x() <<"  "<< t_tmp.y() <<"  "<< t_tmp.z() << endl;
     }
     */
    
    // SFM全局BA优化(full BA)
    // 前面已经求出滑窗内所有帧的路标点的3D位置，对滑动窗口中的所有帧的位姿和3D特征点进行BA优化
    ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
    
    for (int i = 0; i < frame_num; i++)
    {
        // double array for ceres
        c_translation[i][0] = c_Translation[i].x();
        c_translation[i][1] = c_Translation[i].y();
        c_translation[i][2] = c_Translation[i].z();
        c_rotation[i][0] = c_Quat[i].w();
        c_rotation[i][1] = c_Quat[i].x();
        c_rotation[i][2] = c_Quat[i].y();
        c_rotation[i][3] = c_Quat[i].z();
        problem.AddParameterBlock(c_rotation[i], 4, local_parameterization);
        problem.AddParameterBlock(c_translation[i], 3);

        if (i == l)
        {
            problem.SetParameterBlockConstant(c_rotation[i]);
        }
        if (i == l || i == frame_num - 1)
        {
            problem.SetParameterBlockConstant(c_translation[i]);
        }
    }
    
    for (int i = 0; i < feature_num; i++)
    {
        if (sfm_f[i].state != true) {
            continue;
        }
        
        for (int j = 0; j < int(sfm_f[i].observation.size()); j++)
        {
            int l = sfm_f[i].observation[j].first;
            ceres::CostFunction* cost_function = ReprojectionError3D::Create(
                                            sfm_f[i].observation[j].second.x(),
                                            sfm_f[i].observation[j].second.y());
            
            problem.AddResidualBlock(cost_function,
                                     NULL,
                                     c_rotation[l],
                                     c_translation[l],
                                     sfm_f[i].position);	 
        }
    }
    
    TS(SFM_ceres);
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;  // 舒尔补
    // options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 0.5;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    std::cout << "SFM -- " << summary.BriefReport() << "\n";
    
    TE(SFM_ceres);

    if (summary.termination_type == ceres::CONVERGENCE
        || summary.final_cost < 5e-03)
    {
//        cout << "vision only BA converge" << endl;
    }
    else
    {
//        cout << "vision only BA not converge " << endl;
        return false;
    }
    
    for (int i = 0; i < frame_num; i++)
    {
        q[i].w() = c_rotation[i][0]; 
        q[i].x() = c_rotation[i][1]; 
        q[i].y() = c_rotation[i][2]; 
        q[i].z() = c_rotation[i][3]; 
        q[i] = q[i].inverse();
        //cout << "final  q" << " i " << i <<"  " <<q[i].w() << "  " << q[i].vec().transpose() << endl;
    }
    
    for (int i = 0; i < frame_num; i++)
    {
        T[i] = -1 * (q[i] * Vector3d(c_translation[i][0],
                                     c_translation[i][1],
                                     c_translation[i][2]));
        //cout << "final  t" << " i " << i <<"  " << T[i](0) <<"  "<< T[i](1) <<"  "<< T[i](2) << endl;
    }
    
    // BA 优化后的3D坐标？？？
    for (int i = 0; i < (int)sfm_f.size(); i++)
    {
        // TODO: state 为false的坐标点，后续会用到吗？不应该使用state 为false的坐标点
        if (sfm_f[i].state)
        {
            sfm_tracked_points[sfm_f[i].id] = Vector3d(sfm_f[i].position[0],
                                                       sfm_f[i].position[1],
                                                       sfm_f[i].position[2]);
        }
    }
    
    return true;
}

