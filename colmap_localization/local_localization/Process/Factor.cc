#include "Factor.h"

ProjectionCameraMapFactorPoseOnly::ProjectionCameraMapFactorPoseOnly(
                        const Eigen::Vector3d &_map_point, const Eigen::Vector2d &_camera_point,
                        PinholeCamera *pPinholeCamera_):
                     map_point(_map_point), camera_point(_camera_point), pPinholeCamera(pPinholeCamera_)
{
    fx = pPinholeCamera->fx;
    fy = pPinholeCamera->fy;
    cx = pPinholeCamera->cx;
    cy = pPinholeCamera->cy;
}

bool ProjectionCameraMapFactorPoseOnly::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pc(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qc(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Xc = Qc * map_point + Pc;
    double x = Xc(0);
    double y = Xc(1);
    double invz = 1.0 / Xc(2);
    double invz_2 = invz * invz;
    double x_fx = x * fx;
    double y_fy = y * fy;

    residuals[0] = - x_fx * invz - cx + camera_point(0);
    residuals[1] = - y_fy * invz - cy + camera_point(1);

    //std::cout << residuals[0] << "  " << residuals[0] << "\n";

    if(jacobians && jacobians[0]){
        jacobians[0][0] = - invz * fx;
        jacobians[0][1] = 0;
        jacobians[0][2] = x_fx * invz_2;
        jacobians[0][3] = x_fx * y * invz_2;
        jacobians[0][4] = - fx - x_fx * x * invz_2;
        jacobians[0][5] = y * invz * fx;
        jacobians[0][6] = 0;

        jacobians[0][7] = 0;
        jacobians[0][8] = - invz * fy;
        jacobians[0][9] = y_fy * invz_2;
        jacobians[0][10] = fy + y_fy * y * invz_2;
        jacobians[0][11] = - x * y_fy * invz_2;
        jacobians[0][12] = - x * invz * fy;
        jacobians[0][13] = 0;
    }
    return true;
}

void ProjectionCameraMapFactorPoseOnly::check(double **parameters)
{
    double *res = new double[2];
    double **jaco = new double *[1];
    jaco[0] = new double[2 * 7];
    Evaluate(parameters, res, jaco);
    puts("\n check begins");
    puts("my:");
    Eigen::Matrix<double, 2, 7> mine_jacobian;
    for(int i = 0; i < 14; i ++){
        int x = i/7;
        int y = i%7;
        mine_jacobian(x,y) = jaco[0][i];
    }
    std::cout << mine_jacobian << std::endl;

    Eigen::Vector2d residual;
    residual(0) = res[0];
    residual(1) = res[1];

    const double eps = 1e-6;
    Eigen::Matrix<double, 2, 6> num_jacobian;
    for (int k = 0; k < 6; k++){
        Eigen::Vector3d Pc(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qc(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        int a = k / 3, b = k % 3;
        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

        if (a == 0)
            Pc += delta;
        else if (a == 1)
            Qc = (UtilitydeltaQ(delta) * Qc).normalized();

        Eigen::Vector3d Xc = Qc * map_point + Pc;
        double x = Xc(0);
        double y = Xc(1);
        double invz = 1.0 / Xc(2);
        Eigen::Vector2d tmp_residual;
        tmp_residual(0) = - x * invz * fx - cx + camera_point(0);
        tmp_residual(1) = - y * invz * fy - cy + camera_point(1);
        num_jacobian.col(k) = (tmp_residual - residual) / eps;
    }
    puts("numric:");
    std::cout << num_jacobian << std::endl;
}

///////////////////////////

ProjectionCameraMapFactor::ProjectionCameraMapFactor(
                        const Eigen::Vector2d &_camera_point,
                        PinholeCamera *pPinholeCamera_):
                     camera_point(_camera_point), pPinholeCamera(pPinholeCamera_)
{
    fx = pPinholeCamera->fx;
    fy = pPinholeCamera->fy;
    cx = pPinholeCamera->cx;
    cy = pPinholeCamera->cy;
}

bool ProjectionCameraMapFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pc(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qc(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d map_point(parameters[1][0], parameters[1][1], parameters[1][2]);

    Eigen::Vector3d Xc = Qc * map_point + Pc;
    double x = Xc(0);
    double y = Xc(1);
    double invz = 1.0 / Xc(2);
    double x_fx = x * fx;
    double y_fy = y * fy;

    residuals[0] = - x_fx * invz - cx + camera_point(0);
    residuals[1] = - y_fy * invz - cy + camera_point(1);

    //std::cout << residuals[0] << "  " << residuals[0] << "\n";

    if(jacobians){
        double invz_2 = invz * invz;

        if(jacobians[0]){
            jacobians[0][0] = - invz * fx;
            jacobians[0][1] = 0;
            jacobians[0][2] = x_fx * invz_2;
            jacobians[0][3] = x_fx * y * invz_2;
            jacobians[0][4] = - fx - x_fx * x * invz_2;
            jacobians[0][5] = y * invz * fx;
            jacobians[0][6] = 0;

            jacobians[0][7] = 0;
            jacobians[0][8] = - invz * fy;
            jacobians[0][9] = y_fy * invz_2;
            jacobians[0][10] = fy + y_fy * y * invz_2;
            jacobians[0][11] = - x * y_fy * invz_2;
            jacobians[0][12] = - x * invz * fy;
            jacobians[0][13] = 0;
        }
        if(jacobians[1]){
            Eigen::Matrix3d rotation_mat = Qc.matrix();
            double fx_invz = fx * invz;
            double fy_invz = fy * invz;
            double r02_invz2 = rotation_mat(0,2) * invz_2;
            double r12_invz2 = rotation_mat(1,2) * invz_2;
            double r22_invz2 = rotation_mat(2,2) * invz_2;

            jacobians[1][0] = factor_map_point * ( - rotation_mat(0,0) * fx_invz + x_fx * r02_invz2);
            jacobians[1][1] = factor_map_point * ( - rotation_mat(0,1) * fx_invz + x_fx * r12_invz2);
            jacobians[1][2] = factor_map_point * ( - rotation_mat(0,2) * fx_invz + x_fx * r22_invz2);
            jacobians[1][3] = factor_map_point * ( - rotation_mat(0,1) * fy_invz + y_fy * r02_invz2);
            jacobians[1][4] = factor_map_point * ( - rotation_mat(1,1) * fy_invz + y_fy * r12_invz2);
            jacobians[1][5] = factor_map_point * ( - rotation_mat(1,2) * fy_invz + y_fy * r22_invz2);
        }
    }
    return true;
}

double ProjectionCameraMapFactor::EvaluateMine(double* camera_pose, double* map_point)
{
    Eigen::Vector3d Pc(camera_pose[0], camera_pose[1], camera_pose[2]);
    Eigen::Quaterniond Qc(camera_pose[6], camera_pose[3], camera_pose[4], camera_pose[5]);

    Eigen::Vector3d map_point_(map_point[0], map_point[1], map_point[2]);

    Eigen::Vector3d Xc = Qc * map_point_ + Pc;
    double x = Xc(0);
    double y = Xc(1);
    double invz = 1.0 / Xc(2);

    double residuals_1 = - x * fx * invz - cx + camera_point(0);
    double residuals_2 = - y * fy * invz - cy + camera_point(1);

    return (residuals_1 * residuals_1 + residuals_2 * residuals_2);

}

