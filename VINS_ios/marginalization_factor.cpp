//
//  marginalization_factor.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "marginalization_factor.hpp"

void ResidualBlockInfo::Evaluate()
{
    residuals.resize(cost_function->num_residuals());
    
    std::vector<int> block_sizes = cost_function->parameter_block_sizes();
    
    raw_jacobians = new double *[block_sizes.size()];
    jacobians.resize(block_sizes.size());
    
    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
    {
        jacobians[i].resize(cost_function->num_residuals(),
                            block_sizes[i]);
        
        raw_jacobians[i] = jacobians[i].data();
        // dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
    }
    
    cost_function->Evaluate(parameter_blocks.data(),
                            residuals.data(),
                            raw_jacobians);
    
    //std::vector<int> tmp_idx(block_sizes.size());
    //Eigen::MatrixXd tmp(dim, dim);
    //for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
    //{
    //    int size_i = localSize(block_sizes[i]);
    //    Eigen::MatrixXd jacobian_i = jacobians[i].leftCols(size_i);
    //    for (int j = 0, sub_idx = 0; j < static_cast<int>(parameter_blocks.size()); sub_idx += block_sizes[j] == 7 ? 6 : block_sizes[j], j++)
    //    {
    //        int size_j = localSize(block_sizes[j]);
    //        Eigen::MatrixXd jacobian_j = jacobians[j].leftCols(size_j);
    //        tmp_idx[j] = sub_idx;
    //        tmp.block(tmp_idx[i], tmp_idx[j], size_i, size_j) = jacobian_i.transpose() * jacobian_j;
    //    }
    //}
    //Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(tmp);
    //std::cout << saes.eigenvalues() << std::endl;
    //ROS_ASSERT(saes.eigenvalues().minCoeff() >= -1e-6);
    
    if (loss_function)
    {
        double residual_scaling_;
        double alpha_sq_norm_;
        double rho[3];
        double sq_norm = residuals.squaredNorm();
        
        loss_function->Evaluate(sq_norm, rho);
        
        // printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n",
        //         sq_norm, rho[0], rho[1], rho[2]);
        
        double sqrt_rho1_ = sqrt(rho[1]);
        
        if ((sq_norm == 0.0) || (rho[2] <= 0.0))
        {
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_ = 0.0;
        }
        else
        {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }
        
        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
        {
            jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
        }
        
        residuals *= residual_scaling_;
    }
}

MarginalizationInfo::~MarginalizationInfo()
{
    printf("release marginlizationinfo\n");
    
    for (auto it = parameter_block_data.begin();
         it != parameter_block_data.end();
         ++it)
    {
        delete it->second;  // TODO: 与Mono不一致
    }
    
    for (int i = 0; i < (int)factors.size(); i++)
    {
        delete[] factors[i]->raw_jacobians;
        delete factors[i]->cost_function;
        delete factors[i];
    }
}

void MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo *residual_block_info)
{
    factors.emplace_back(residual_block_info);
    
    std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;
    std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();
    
    // Global
    for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++)
    {
        double *addr = parameter_blocks[i];
        int size = parameter_block_sizes[i];
        
        parameter_block_size[reinterpret_cast<long>(addr)] = size;
    }
    
    // Local 不包含drop_set
    for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++)
    {
        double *addr = parameter_blocks[residual_block_info->drop_set[i]];
        parameter_block_idx[reinterpret_cast<long>(addr)] = 0;
    }
}

/**
 * 根据各个测量模型Evaluate() 计算残差；
 * 各个参数块拷贝到统一的内存（parameter_block_data）中
 *
 * 得到每个残差项（cost_function）对应的参数块(parameter_blocks)、jacobians、残差值（residuals）
 */
void MarginalizationInfo::preMarginalize()
{
    for (auto it : factors)
    {
        it->Evaluate();
        
        std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();
        
        for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
        {
            long addr = reinterpret_cast<long>(it->parameter_blocks[i]);
            int size = block_sizes[i];
            
            // 没有找到此参数块，则加入到parameter_block_data
            if (parameter_block_data.find(addr) == parameter_block_data.end())
            {
                double *data = new double[size];
                memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
                
                parameter_block_data[addr] = data;
            }
        }
    }
}

// Pose: p_3 & q_4  ->  p_3 & q_3
int MarginalizationInfo::localSize(int size) const
{
    return size == 7 ? 6 : size;
}

// Pose: p_3 & q_3  ->  p_3 & q_4
int MarginalizationInfo::globalSize(int size) const
{
    return size == 6 ? 7 : size;
}

/**
 * 多线程构造边缘化矩阵 H & b；(Hx = b)
 */
void * ThreadsConstructA(void* threadsstruct)
{
    ThreadsStruct* p = ((ThreadsStruct*)threadsstruct);
    
    for (auto it : p->sub_factors)
    {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
        {
            int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
            int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
            if (size_i == 7)  // Pose: p_3 & q_4  ->  p_3 & q_3
            {
                size_i = 6;
            }
            
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
                if (size_j == 7)
                {
                    size_j = 6;
                }
                
                // 计算9.9节中的公式（9-25）H矩阵[J^T * J]
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                if (i == j)
                {
                    // 对角线上的值
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                }
                else
                {
                    // 对角线两侧 对称的值
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            
            // 对应9.9节中的公式（9-25）的右侧（残差）
            p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    
    return threadsstruct;
}

/**
 * 利用多线程构造稀疏矩阵H
 * 利用舒尔补消元简化稀疏矩阵求解过程
 * 滑动窗口中参数块存储地址调整
 */
/** 边缘化
 * 多线程构造Hx=b的结构，H是边缘化后的结果（根据parameter_block_idx里面的标记）
 * 使用Schur complement简化计算过程
 * First Esitimate Jacobian，在X0处线性化
 *
 * 多线程计整个先验项的参数块，雅可比矩阵和残差值，对应舒尔补公式（9-7）
 */
void MarginalizationInfo::marginalize()
{
    int pos = 0;
    for (auto &it : parameter_block_idx)
    {
        it.second = pos;
        pos += localSize(parameter_block_size[it.first]);
    }
    
    margParamCnt_m = pos;
    
    for (const auto &it : parameter_block_size)
    {
        if (parameter_block_idx.find(it.first) == parameter_block_idx.end())
        {
            parameter_block_idx[it.first] = pos;
            pos += localSize(it.second);
        }
    }
    
    keepParamCnt_n = pos - margParamCnt_m;
    
//    printf("marginalization, pos: %d, m: %d, n: %d, size: %d",
//           pos, margParamCnt_m, keepParamCnt_n, (int)parameter_block_idx.size());
    if(margParamCnt_m == 0)
    {
//        valid = false;
        printf("unstable tracking......\n");
        return;
    }
    
    Eigen::MatrixXd A(pos, pos);
    Eigen::VectorXd b(pos);
    A.setZero();
    b.setZero();
    
    /*
     for (auto it : factors)
     {
     for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
     {
     int idx_i = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
     int size_i = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])]);
     Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
     for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
     {
     int idx_j = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
     int size_j = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])]);
     Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
     if (i == j)
     A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
     else
     {
     A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
     A.block(idx_j, idx_i, size_j, size_i) = A.block(idx_i, idx_j, size_i, size_j).transpose();
     }
     }
     b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
     }
     }
     ROS_INFO("summing up costs %f ms", t_summing.toc());
     */
    //multi thread
    
    pthread_t tids[NUM_THREADS];
    ThreadsStruct threadsstruct[NUM_THREADS];
    int i = 0;
    
    for (auto it : factors)
    {
        threadsstruct[i].sub_factors.push_back(it);
        i++;
        i = i % NUM_THREADS;
    }
    
    for (int i = 0; i < NUM_THREADS; i++)
    {
        threadsstruct[i].A = Eigen::MatrixXd::Zero(pos, pos);
        threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
        threadsstruct[i].parameter_block_size = parameter_block_size;
        threadsstruct[i].parameter_block_idx = parameter_block_idx;
        
        int ret = pthread_create(&tids[i],
                                 NULL,
                                 ThreadsConstructA,
                                 (void*)&(threadsstruct[i]));
        if (ret != 0)
        {
            printf("marginalize pthread_create error\n");
            assert(false);
        }
    }
    
    // Ax = b
    for (int i = NUM_THREADS - 1; i >= 0; i--)
    {
        pthread_join(tids[i], NULL);
        
        A += threadsstruct[i].A;
        b += threadsstruct[i].b;
    }
    
    // ROS_INFO("A diff %f , b diff %f ", (A - tmp_A).sum(), (b - tmp_b).sum());
    
    Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, margParamCnt_m, margParamCnt_m)
                                 + A.block(0, 0, margParamCnt_m, margParamCnt_m).transpose());
    
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
    
    // ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue %f", saes.eigenvalues().minCoeff());
    
    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() * saes.eigenvectors().transpose();
    
    // printf("error1: %f\n", (Amm * Amm_inv - Eigen::MatrixXd::Identity(m, m)).sum());
    
    // 对应9.5节中的图9-16
    Eigen::VectorXd bmm = b.segment(0, margParamCnt_m);
    Eigen::MatrixXd Amr = A.block(0, margParamCnt_m, margParamCnt_m, keepParamCnt_n);
    Eigen::MatrixXd Arm = A.block(margParamCnt_m, 0, keepParamCnt_n, margParamCnt_m);
    Eigen::MatrixXd Arr = A.block(margParamCnt_m, margParamCnt_m, keepParamCnt_n, keepParamCnt_n);
    Eigen::VectorXd brr = b.segment(margParamCnt_m, keepParamCnt_n);
    
    // 对应9.9节中的公式（9-30）
    A = Arr - Arm * Amm_inv * Amr;
    b = brr - Arm * Amm_inv * bmm;
    
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));
    
    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
    
    // FEJ First Estimate Jacobians，以后计算关于先验的误差和Jacobian都在边缘化的这个线性点展开，有点费劲，参考资料：
    // http://blog.csdn.net/heyijia0327/article/details/53707261 DSO 中的Windowed Optimization
    // https://mp.weixin.qq.com/s?__biz=MzI5MTM1MTQwMw==&mid=2247486797&idx=1&sn=6ae98c0c52ce74ddb5cdc17f3e0113b7&chksm=ec10b349db673a5fdc7c9db385eb39efc0a9724519a8ece8ce5dbe504d33e4ba099dafbcc65f&mpshare=1&scene=24&srcid=1113T4dVqwLiyL4XDMDerW4Z#rd
    // OKVIS理论推导（下） SLAM中的marginalization 和 Schur complement
    // http://blog.csdn.net/heyijia0327/article/details/52822104
    // Diagonal 对角线；对应9.9节中的公式（9-40）
    linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
    
    //std::cout << A << std::endl
    //          << std::endl;
    //std::cout << linearized_jacobians << std::endl;
    //printf("error2: %f %f\n", (linearized_jacobians.transpose() * linearized_jacobians - A).sum(),
    //      (linearized_jacobians.transpose() * linearized_residuals - b).sum());
}

std::vector<double *> MarginalizationInfo::getParameterBlocks(
                            std::unordered_map<long, double *> &addr_shift)
{
    std::vector<double *> keep_block_addr;
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();
    
    for (const auto &it : parameter_block_idx)
    {
        if (it.second >= margParamCnt_m)
        {
            keep_block_size.push_back(parameter_block_size[it.first]);
            keep_block_idx.push_back(parameter_block_idx[it.first]);
            keep_block_data.push_back(parameter_block_data[it.first]);
            keep_block_addr.push_back(addr_shift[it.first]);
        }
    }
    
    sum_block_size = std::accumulate(std::begin(keep_block_size),
                                     std::end(keep_block_size), 0);
    
    return keep_block_addr;
}

MarginalizationFactor::MarginalizationFactor(MarginalizationInfo* _marginalization_info)
                                : marginalization_info(_marginalization_info)
{
    int cnt = 0;
    for (auto it : marginalization_info->keep_block_size)
    {
        mutable_parameter_block_sizes()->push_back(it);
        cnt += it;
    }
    
    // printf("residual size: %d, %d\n", cnt, n);
    set_num_residuals(marginalization_info->keepParamCnt_n);
}

/**
 * 先验残差
 * 1.根据FEJ，固定线性化的点，更新这里的residuals和Jacobian
 */
bool MarginalizationFactor::Evaluate(double const *const *parameters,
                                     double *residuals,
                                     double **jacobians) const
{
    //printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(), num_residuals());
    //for (int i = 0; i < static_cast<int>(keep_block_size.size()); i++)
    //{
    //    //printf("unsigned %x\n", reinterpret_cast<unsigned long>(parameters[i]));
    //    //printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
    //printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
    //printf("residual %x\n", reinterpret_cast<long>(residuals));
    //}
    int n = marginalization_info->keepParamCnt_n;
    int m = marginalization_info->margParamCnt_m;
    Eigen::VectorXd dx(n);
    
    for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
    {
        int size = marginalization_info->keep_block_size[i];
        int idx = marginalization_info->keep_block_idx[i] - m;
        Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size);
       
        if (size != 7)
        {
            dx.segment(idx, size) = x - x0;
        }
        else
        {
            dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
            dx.segment<3>(idx + 3) = 2.0 * Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            
            if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).w() >= 0))
            {
                dx.segment<3>(idx + 3) = 2.0 * -Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            }
        }
    }
    
    Eigen::Map<Eigen::VectorXd>(residuals, n) = marginalization_info->linearized_residuals
                                            + marginalization_info->linearized_jacobians * dx;
    if (jacobians)
    {
        for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
        {
            if (jacobians[i])
            {
                int size = marginalization_info->keep_block_size[i];
                int local_size = marginalization_info->localSize(size);
                int idx = marginalization_info->keep_block_idx[i] - m;
                
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], n, size);
                
                jacobian.setZero();
                jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);
            }
        }
    }
    
    return true;
}

