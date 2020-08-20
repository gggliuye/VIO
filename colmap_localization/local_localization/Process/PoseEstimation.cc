#include "PoseEstimation.h"


namespace BASTIAN
{

bool EstimatePosePnPRansac(CurrentFrame* pCurrentFrame, Eigen::Vector4d &qvec, Eigen::Vector3d &tvec)
{
    int N = pCurrentFrame->CountMatches();
    //std::cout << "==> Current frame found " << N << " matches.\n";

    if(N < 30){
        return false;
    }

    cv::Mat cvCalibration = cv::Mat::zeros(3,3,CV_64FC1);
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3;j ++)
            cvCalibration.at<double>(i,j) = pCurrentFrame->mCalibration(i,j);

    cv::Mat cvDistortionCoeff = cv::Mat::zeros(5,1,CV_64FC1);

    std::vector<cv::Point3f> obj_pts;
    std::vector<cv::Point2f> img_pts;
    obj_pts.reserve(N);
    img_pts.reserve(N);

    std::vector<FeatureKeypoint> &vFeatureKeypoints = pCurrentFrame->vFeatureKeypoints;
    for(int i = 0; i < vFeatureKeypoints.size(); i++){
        if(pCurrentFrame->vPointMatches[i].flag){
            Eigen::Vector3d &mWorld = pCurrentFrame->vPointMatches[i].mWorld;
            cv::Point3f cvWorld(mWorld(0), mWorld(1), mWorld(2));
            obj_pts.push_back(cvWorld);
            img_pts.push_back(cv::Point2f(vFeatureKeypoints[i].x, vFeatureKeypoints[i].y));
        }
    }

    //std::cout << obj_pts.size() << " " << img_pts.size() << std::endl;

    // set the original pose as the initial pose esitmation for PnP RANSAC
    cv::Mat pnpRotationEuler;
    cv::Mat pnpTranslation = cv::Mat::zeros(3,1,CV_64FC1);

    cv::Mat rotMat_init = cv::Mat::zeros(3,3,CV_64FC1);
    Eigen::Quaterniond quaternion_init(qvec(0), qvec(1), qvec(2), qvec(3)); 
    Eigen::Matrix3d rotMat_eigen = quaternion_init.matrix();
    for(int i= 0; i<3;i++){
        for(int j= 0; j <3;j++){
            rotMat_init.at<double>(i,j) = rotMat_eigen(i,j);
        }
        pnpTranslation.at<double>(i) = tvec(i);
    }

    cv::Rodrigues(rotMat_init, pnpRotationEuler);

    cv::Mat inliers;
    bool ret = cv::solvePnPRansac(obj_pts, img_pts, cvCalibration, cvDistortionCoeff, 
                                      pnpRotationEuler, pnpTranslation,
                                      true, 300, 8.0, 0.99, inliers, cv::SOLVEPNP_P3P); // SOLVEPNP_P3P , SOLVEPNP_ITERATIVE

    if(!ret){
        return ret;
    }

    cv::Mat rotMat;
    cv::Rodrigues(pnpRotationEuler, rotMat);
        
    Eigen::Matrix3d mRotMat;
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3;j ++)
            mRotMat(i,j) = rotMat.at<double>(i,j);
    Eigen::Quaterniond mQ(mRotMat);
    qvec << mQ.w(), mQ.x(), mQ.y(), mQ.z();
    tvec << pnpTranslation.at<double>(0), pnpTranslation.at<double>(1), pnpTranslation.at<double>(2);

    int N_inlier = inliers.rows;
    std::vector<cv::Point3f> obj_pts_inlier;
    std::vector<cv::Point2f> img_pts_inlier;
    obj_pts.reserve(N_inlier);
    img_pts.reserve(N_inlier);

    //std::cout << "==> RANSAC found " << N_inlier << " inlier points." << std::endl;

    for(int i= 0; i < N_inlier; i++){
        int id_inlier = inliers.at<int>(i);
        obj_pts_inlier.push_back(obj_pts[id_inlier]);
        img_pts_inlier.push_back(img_pts[id_inlier]);
    }
    
    pCurrentFrame->last_matched_n_inliers = N_inlier;
    return OptimizeCameraPose(obj_pts_inlier, img_pts_inlier, pCurrentFrame, qvec, tvec);
    //return OptimizeCameraPose(obj_pts, img_pts, pCurrentFrame, qvec, tvec);
}


void GetPoseTcwArray(Eigen::Matrix4d &Tcw_curr, double *pose)
{
    Eigen::Matrix3d rotation_matrix = Tcw_curr.block(0,0,3,3);
    Eigen::Quaterniond qe(rotation_matrix);
    pose[0] = Tcw_curr(0,3); pose[1] = Tcw_curr(1,3); pose[2] = Tcw_curr(2,3);
    pose[3] = qe.x(); pose[4] = qe.y(); pose[5] = qe.z(); pose[6] = qe.w();
}

void GetPoseTcwEigen(Eigen::Matrix4d &Tcw_curr, double *pose)
{
    Tcw_curr.setIdentity();
    Eigen::Quaterniond qc(pose[6], pose[3], pose[4], pose[5]);
    Tcw_curr.block(0,0,3,3) = qc.matrix();
    Tcw_curr(0,3) = pose[0]; Tcw_curr(1,3) = pose[1]; Tcw_curr(2,3) = pose[2];
}

bool OptimizeCameraPose(std::vector<cv::Point3f> &obj_pts, std::vector<cv::Point2f> &img_pts,
                       CurrentFrame* pCurrentFrame, Eigen::Vector4d &qvec, Eigen::Vector3d &tvec)
{
    Eigen::Matrix4d Tcw_curr;

    Eigen::Quaterniond quaternion_init(qvec(0), qvec(1), qvec(2), qvec(3)); 
    Tcw_curr.block(0,0,3,3) = quaternion_init.matrix();
    Tcw_curr(0,3) = tvec(0); Tcw_curr(1,3) = tvec(1); Tcw_curr(2,3) = tvec(2);

    PinholeCamera* pPinholeCamera = new PinholeCamera(pCurrentFrame->mCalibration(0,0), pCurrentFrame->mCalibration(1,1),
              pCurrentFrame->mCalibration(0,2), pCurrentFrame->mCalibration(1,2), pCurrentFrame->cols, pCurrentFrame->rows);

    bool ret = OptimizeCameraPose(obj_pts, img_pts, Tcw_curr, pPinholeCamera);

    if(ret){
        Eigen::Matrix3d mRotMat = Tcw_curr.block(0,0,3,3);
        Eigen::Quaterniond mQ(mRotMat);
        qvec << mQ.w(), mQ.x(), mQ.y(), mQ.z();
        tvec << Tcw_curr(0,3), Tcw_curr(1,3), Tcw_curr(2,3);
    }

    return ret;
}

bool OptimizeCameraPose(std::vector<cv::Point3f> &obj_pts, std::vector<cv::Point2f> &img_pts, 
                        Eigen::Matrix4d &Tcw_curr, PinholeCamera* pPinholeCamera)
{
    bool use_robust_fcn = true;
    bool check_jacobian = false;

    double camera_pose[7];
    GetPoseTcwArray(Tcw_curr, camera_pose);

    ceres::Problem problem;
    ceres::LossFunctionWrapper* loss_function = new ceres::LossFunctionWrapper(new ceres::CauchyLoss(1.5), ceres::TAKE_OWNERSHIP);

    ceres::LocalParameterization *local_parameterization = new PoseLyParameterization();
    problem.AddParameterBlock(camera_pose, 7, local_parameterization);

    for(size_t i = 0, iend = obj_pts.size() ; i < iend ; i++){
        Eigen::Vector2d camera_point(img_pts[i].x, img_pts[i].y);
        Eigen::Vector3d map_point(obj_pts[i].x, obj_pts[i].y, obj_pts[i].z);

        ProjectionCameraMapFactorPoseOnly *f_camera_pose = new ProjectionCameraMapFactorPoseOnly(map_point, camera_point,
                         pPinholeCamera);

        if(check_jacobian){
            double **jaco = new double *[1];
            jaco[0] = camera_pose;
            f_camera_pose->check(jaco);
        }

        if(use_robust_fcn)
            problem.AddResidualBlock(f_camera_pose, loss_function, camera_pose);
        else
            problem.AddResidualBlock(f_camera_pose, NULL, camera_pose);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    //options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 40;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.BriefReport() << std::endl;
    //printf("Iterations : %d \n", static_cast<int>(summary.iterations.size()));

    GetPoseTcwEigen(Tcw_curr, camera_pose);

    return summary.IsSolutionUsable();
}



} //namespace
