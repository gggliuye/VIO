#include "../thirdparty/Sophus/sophus/se3.hpp"

#include "backend/edge_mapfusion.h"
#include "backend/vertex_pose.h"
#include "backend/vertex_scale.h"

#include <iostream>

#define USE_SO3_JACOBIAN 1

namespace myslam {
namespace backend {

Qd deltaQ(const Vec3 &theta)
{
    Eigen::Quaternion<double> dq;
    Eigen::Matrix<double, 3, 1> half_theta = theta;
    half_theta /= 2.0;
    dq.w() = 1.0;
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
}

void EdgeMapFusion::ComputeResidual() 
{
    VecX param_pose = verticies_[0]->Parameters();
    Qd Q_relative(param_pose[6], param_pose[3], param_pose[4], param_pose[5]);
    Vec3 P_relative = param_pose.head<3>();

    double scale = verticies_[1]->Parameters()[0];

    residual_.setZero();

    Vec3 vectorErr = - pts_j_ + scale * (Q_relative * pts_i_ + P_relative);
    residual_ = vectorErr;

}

void EdgeMapFusion::ComputeJacobians() 
{
    VecX param_pose = verticies_[0]->Parameters();
    Qd Q_relative(param_pose[6], param_pose[3], param_pose[4], param_pose[5]);
    Vec3 P_relative = param_pose.head<3>();

    double scale = verticies_[1]->Parameters()[0];


    Eigen::Matrix<double, 3, 1> jacobian_scale;
    jacobian_scale = Q_relative * pts_i_+ P_relative;


    Eigen::Matrix<double, 3, 6> jacobian_relative_pose;
    jacobian_relative_pose.setZero();
    jacobian_relative_pose.leftCols<3>() = scale * Eigen::Matrix3d::Identity();
    jacobian_relative_pose.rightCols<3>() = - scale * (Q_relative * Sophus::SO3d::hat(pts_i_));

    jacobians_[0] = jacobian_relative_pose;
    jacobians_[1] = jacobian_scale;

    //CheckJacobian();
}

void EdgeMapFusion::CheckJacobian()
{
    VecX param_pose = verticies_[0]->Parameters();
    Qd Q_relative(param_pose[6], param_pose[3], param_pose[4], param_pose[5]);
    Vec3 P_relative = param_pose.head<3>();

    double scale = verticies_[1]->Parameters()[0];

    const double eps = 1e-6;

    std::cout << "[DEBUG] Check scale jacobian :" << std::endl;
    
    Vec3 vectorErr = - pts_j_ + (scale + eps) * (Q_relative * pts_i_ + P_relative);
    Eigen::Matrix<double, 3, 1> jacobian_scale = (vectorErr - residual_) / eps;

    std::cout << "        numerical Jacobian scale : " << jacobian_scale.transpose() << std::endl;
    std::cout << "        theorical Jacobian scale : " << jacobians_[1].transpose() << std::endl;
    

    std::cout << "[DEBUG] Check translation jacobian :" << std::endl;
    Eigen::Vector3d pose_delta(eps, 0, 0);
    vectorErr = - pts_j_ + (scale) * (Q_relative * pts_i_ + P_relative + pose_delta);
    Eigen::Matrix<double, 3, 1> jacobian_num = (vectorErr - residual_) / eps;
    
    std::cout << "        numerical Jacobian t.x : " << jacobian_num.transpose() << std::endl;
    std::cout << "        theorical Jacobian t.x : " << jacobians_[0].col(0).transpose() << std::endl;

    pose_delta << 0, eps, 0;
    vectorErr = - pts_j_ + (scale) * (Q_relative * pts_i_ + P_relative + pose_delta);
    jacobian_num = (vectorErr - residual_) / eps;
    
    std::cout << "        numerical Jacobian t.y : " << jacobian_num.transpose() << std::endl;
    std::cout << "        theorical Jacobian t.y : " << jacobians_[0].col(1).transpose() << std::endl;

    pose_delta << 0, 0, eps;
    vectorErr = - pts_j_ + (scale) * (Q_relative * pts_i_ + P_relative + pose_delta);
    jacobian_num = (vectorErr - residual_) / eps;
    
    std::cout << "        numerical Jacobian t.z : " << jacobian_num.transpose() << std::endl;
    std::cout << "        theorical Jacobian t.z : " << jacobians_[0].col(2).transpose() << std::endl;

    std::cout << "[DEBUG] Check rotation jacobian :" << std::endl;
    pose_delta << eps, 0, 0;

    vectorErr = - pts_j_ + (scale) * (Q_relative * deltaQ(pose_delta) * pts_i_ + P_relative);
    jacobian_num = (vectorErr - residual_) / eps;
    
    std::cout << "        numerical Jacobian q.x : " << jacobian_num.transpose() << std::endl;
    std::cout << "        theorical Jacobian q.x : " << jacobians_[0].col(3).transpose() << std::endl;

    pose_delta << 0, eps, 0;
    vectorErr = - pts_j_ + (scale) * (Q_relative * deltaQ(pose_delta) * pts_i_ + P_relative);
    jacobian_num = (vectorErr - residual_) / eps;
    
    std::cout << "        numerical Jacobian t.y : " << jacobian_num.transpose() << std::endl;
    std::cout << "        theorical Jacobian t.y : " << jacobians_[0].col(4).transpose() << std::endl;

    pose_delta << 0, 0, eps;
    vectorErr = - pts_j_ + (scale) * (Q_relative * deltaQ(pose_delta) * pts_i_ + P_relative);
    jacobian_num = (vectorErr - residual_) / eps;
    
    std::cout << "        numerical Jacobian t.z : " << jacobian_num.transpose() << std::endl;
    std::cout << "        theorical Jacobian t.z : " << jacobians_[0].col(5).transpose() << std::endl;

}


}   // namespace backend
}   // namespace myslam
