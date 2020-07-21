#ifndef G_OPTIMIZATION_H
#define G_OPTIMIZATION_H

#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "LoopClosure/GFactors.h"
#include "LoopClosure/KeyFrameMapPoint.h"


#define PI (3.1415926535897932346f)

// success
void GOptimization(GKeyFrame *pKF, bool bRobust = false)
{
    cv::Mat &cvT = pKF->Tcw;
    Eigen::Matrix<double,3,3> rotation_matrix;
    rotation_matrix << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
                       cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
                       cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Quaterniond qe(rotation_matrix);
    double cameraPose[7] = {cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3),
                              qe.x(), qe.y(), qe.z(), qe.w()};

    ceres::Problem problem;
    ceres::LossFunctionWrapper* loss_function = new ceres::LossFunctionWrapper(new ceres::HuberLoss(1.0), ceres::TAKE_OWNERSHIP);

    for(size_t i=0, iend = pKF->mvpMapPoints.size() ; i < iend ; i++){
        GMapPoint* pMP = pKF->mvpMapPoints[i];
        if(!pMP)
            continue;

        const cv::KeyPoint &kp = pKF->keypoints[i];
        cv::Mat Pos = pMP->GetWorldPos();
        cv::Point3f map_point(Pos.at<float>(0),Pos.at<float>(1),Pos.at<float>(2));
        cv::Point2f camera_point(kp.pt.x, kp.pt.y);
        ceres::CostFunction* costfunction = new ceres::AutoDiffCostFunction<cost_function_pnp_quaternion,2,7>(
                       new cost_function_pnp_quaternion(map_point,camera_point,pKF->fx, pKF->fy, pKF->cx, pKF->cy));
        if(bRobust)
            problem.AddResidualBlock(costfunction, loss_function, cameraPose);
        else
            problem.AddResidualBlock(costfunction, NULL, cameraPose);   
    }
 

    ceres::Solver::Options options;
    options.linear_solver_type=ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    if(bRobust){
       options.max_num_iterations = 20; 
    } else {
       options.max_num_iterations = 10; 
    }
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);
    std::cout << summary.BriefReport() << std::endl;

    Eigen::Quaterniond Qc(cameraPose[6], cameraPose[3], cameraPose[4], cameraPose[5]);
    Eigen::Matrix3d qmat = Qc.matrix();

    cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            Tcw.at<float>(i,j) = qmat(i,j);
        }
    }
    for(int i=0;i<3;i++){
        Tcw.at<float>(i,3) = cameraPose[i];
    }

    pKF->SetPose(Tcw);
}

// success
int GOptimizationRANSAC(GKeyFrame *pKF, float minLoss = 4.0)
{
    // Firstly do a normal optimization with robust loss function to have residence against outliers
    cv::Mat &cvT = pKF->Tcw;
    Eigen::Matrix<double,3,3> rotation_matrix;
    rotation_matrix << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
                       cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
                       cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Quaterniond qe(rotation_matrix);
    double cameraPose[7] = {cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3),
                              qe.x(), qe.y(), qe.z(), qe.w()};

    ceres::Problem *problem = new ceres::Problem;
    ceres::LossFunctionWrapper* loss_function = new ceres::LossFunctionWrapper(new ceres::HuberLoss(1.0), ceres::TAKE_OWNERSHIP);

    for(size_t i=0, iend = pKF->mvpMapPoints.size() ; i < iend ; i++){
        GMapPoint* pMP = pKF->mvpMapPoints[i];
        if(!pMP)
            continue;
        const cv::KeyPoint &kp = pKF->keypoints[i];
        cv::Mat Pos = pMP->GetWorldPos();
        cv::Point3f map_point(Pos.at<float>(0),Pos.at<float>(1),Pos.at<float>(2));
        cv::Point2f camera_point(kp.pt.x, kp.pt.y);
        ceres::CostFunction* costfunction = new ceres::AutoDiffCostFunction<cost_function_pnp_quaternion,2,7>(
                       new cost_function_pnp_quaternion(map_point,camera_point,pKF->fx, pKF->fy, pKF->cx, pKF->cy));
        problem->AddResidualBlock(costfunction, loss_function, cameraPose);
    }
 
    ceres::Solver::Options options;
    options.linear_solver_type=ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 20; 
    //options.max_num_iterations = 10; 
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem,&summary);
    std::cout << summary.BriefReport() << std::endl;

    int nGood = 0;
    // Second Optimization : evaluate the loss of each point and reject points with large error
    // Then make another optimization without robust loss function
    problem = new ceres::Problem;
    for(size_t i=0, iend = pKF->mvpMapPoints.size() ; i < iend ; i++){
        GMapPoint* pMP = pKF->mvpMapPoints[i];
        if(!pMP)
            continue;

        const cv::KeyPoint &kp = pKF->keypoints[i];
        cv::Mat Pos = pMP->GetWorldPos();
        cv::Point3f map_point(Pos.at<float>(0),Pos.at<float>(1),Pos.at<float>(2));
        cv::Point2f camera_point(kp.pt.x, kp.pt.y);
        float loss = EvaluatePnP(cameraPose,map_point,camera_point,pKF->fx, pKF->fy, pKF->cx, pKF->cy);
        if(loss > minLoss){
            // remove this match
            pKF->mvpMapPoints[i] = NULL;
            continue;
        }

        ceres::CostFunction* costfunction = new ceres::AutoDiffCostFunction<cost_function_pnp_quaternion,2,7>(
                       new cost_function_pnp_quaternion(map_point,camera_point,pKF->fx, pKF->fy, pKF->cx, pKF->cy));
        problem->AddResidualBlock(costfunction, NULL, cameraPose);
        nGood++;
    }

    options.max_num_iterations = 10; 
    ceres::Solve(options, problem, &summary);
    std::cout << summary.BriefReport() << std::endl;


    Eigen::Quaterniond Qc(cameraPose[6], cameraPose[3], cameraPose[4], cameraPose[5]);
    Eigen::Matrix3d qmat = Qc.matrix();

    cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            Tcw.at<float>(i,j) = qmat(i,j);
        }
    }
    for(int i=0;i<3;i++){
        Tcw.at<float>(i,3) = cameraPose[i];
    }

    pKF->SetPose(Tcw);

    return nGood;
}


// failed , not correct
void GOptimizationF(GKeyFrame *pKF)
{

    cv::Mat &cvT = pKF->Tcw;
    Eigen::Matrix<double,3,3> rotation_matrix;
    rotation_matrix << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
                       cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
                       cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Vector3d euler = rotation_matrix.eulerAngles(2,1,0);

    //Eigen::Quaterniond qe(rotation_matrix);

    double cere_rot[3] = {euler(0), euler(1), euler(2)};
    double cere_tranf[3] = {cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3)};

    std::cout << " r : " << cere_rot[0] << " " << cere_rot[1] << " " << cere_rot[2] << " t :" << cere_tranf[0] << " " << cere_tranf[1] << " " << cere_tranf[2] << " \n";

    ceres::Problem problem;

    for(size_t i=0, iend = pKF->mvpMapPoints.size() ; i < iend ; i++){

        GMapPoint* pMP = pKF->mvpMapPoints[i];
        if(!pMP)
            continue;

        const cv::KeyPoint &kp = pKF->keypoints[i];
        cv::Mat Pos = pMP->GetWorldPos();
        cv::Point3f map_point(Pos.at<float>(0),Pos.at<float>(1),Pos.at<float>(2));
        cv::Point2f camera_point(kp.pt.x, kp.pt.y);
        ceres::CostFunction* costfunction = new ceres::AutoDiffCostFunction<cost_function_define,2,3,3>(
                       new cost_function_define(map_point,camera_point,pKF->fx, pKF->fy, pKF->cx, pKF->cy));
        problem.AddResidualBlock(costfunction, NULL, cere_rot, cere_tranf);   
    }
 

    ceres::Solver::Options option;
    option.linear_solver_type=ceres::DENSE_SCHUR;
    option.minimizer_progress_to_stdout=true;
    ceres::Solver::Summary summary;
    ceres::Solve(option,&problem,&summary);
    std::cout<<summary.BriefReport()<<std::endl;

    std::cout<<"----------------optional after--------------------"<<std::endl;

    cv::Mat cam_3d = (cv::Mat_<double> ( 3,1 ) << cere_rot[0],cere_rot[1],cere_rot[2]);
    cv::Mat cam_9d;
    cv::Rodrigues (cam_3d, cam_9d); 

    cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            Tcw.at<float>(i,j) = cam_9d.at<double>(i,j);
        }
    }
    for(int i=0;i<3;i++){
        Tcw.at<float>(i,3) = cere_tranf[i];
    }

    pKF->SetPose(Tcw);

}


// failed , jacobian not correct
void GOptimizationT(GKeyFrame *pKF)
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);

    cv::Mat &cvT = pKF->Tcw;
    Eigen::Matrix<double,3,3> rotation_matrix;
    rotation_matrix << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
                       cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
                       cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Quaterniond qe(rotation_matrix);
    double cameraPose[7] = {cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3),
                              qe.x(), qe.y(), qe.z(), qe.w()};
    
    
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(cameraPose, 7, local_parameterization);

    int f_m_cnt = 0;
    for(size_t i=0, iend = pKF->mvpMapPoints.size() ; i < iend ; i++){
        GMapPoint* pMP = pKF->mvpMapPoints[i];
        if(pMP){
            const cv::KeyPoint &kp = pKF->keypoints[i];
            cv::Mat Pos = pMP->GetWorldPos();
            Eigen::Vector3d map_point(Pos.at<float>(0),Pos.at<float>(1),Pos.at<float>(2));
            Eigen::Vector2d camera_point(kp.pt.x, kp.pt.y);

            ProjectionCameraMapFactor *f_cam = new ProjectionCameraMapFactor(map_point, camera_point,
                             pKF->fx, pKF->fy, pKF->cx, pKF->cy);
            double **jaco = new double *[1];
            jaco[0] = cameraPose;
            f_cam->check(jaco);
            problem.AddResidualBlock(f_cam, loss_function, cameraPose);
            f_m_cnt++;
        }
    }
    printf("visual measurement count: %d \n", f_m_cnt);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 10;
    options.minimizer_progress_to_stdout = true;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    printf("Iterations : %d \n", static_cast<int>(summary.iterations.size()));

    Eigen::Quaterniond Qc(cameraPose[6], cameraPose[3], cameraPose[4], cameraPose[5]);
    Eigen::Matrix3d qmat = Qc.matrix();

    cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            Tcw.at<float>(i,j) = qmat(i,j);
        }
    }
    for(int i=0;i<3;i++){
        Tcw.at<float>(i,3) = cameraPose[i];
    }

    pKF->SetPose(Tcw);

}





#endif // #ifndef G_OPTIMIZATION_H
