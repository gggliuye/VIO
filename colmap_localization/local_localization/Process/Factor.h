#ifndef FACTOR_PNP_H
#define FACTOR_PNP_H

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

class PinholeCamera
{
public:
    PinholeCamera(double fx_, double fy_, double cx_, double cy_, int width_, int height_):
        fx(fx_), fy(fy_), cx(cx_), cy(cy_),width(width_), height(height_){
            inv_fx = 1.0/fx;
            inv_fy = 1.0/fy;
        }

    double fx;
    double fy;
    double cx;
    double cy;
    int width;
    int height;
    double inv_fx;
    double inv_fy;
};

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> UtilitydeltaQ(const Eigen::MatrixBase<Derived> &theta)
{
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
}


class PoseLyParameterization : public ceres::LocalParameterization
{
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };

    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const
    {
        Eigen::Map<const Eigen::Vector3d> _p(x);
        Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

        Eigen::Map<const Eigen::Vector3d> dp(delta);

        Eigen::Quaterniond dq = UtilitydeltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

        Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
        Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

        p = _p + dp;
        q = (dq * _q).normalized();

        return true;
    }

    virtual bool ComputeJacobian(const double *x, double *jacobian) const
    {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
        j.topRows<6>().setIdentity();
        j.bottomRows<1>().setZero();

        return true;
    }

};

// template<int kNumResiduals, int N0 = 0, int N1 = 0, int N2 = 0, int N3 = 0, int N4 = 0,
//                             int N5 = 0, int N6 = 0, int N7 = 0, int N8 = 0, int N9 = 0>
// our number of residual should be 2 (the distance in 2D image space)
//     -> reproject the map point into image frame
// we only optimize one node, which is the camera pose, whose dimension is 7
class ProjectionCameraMapFactorPoseOnly : public ceres::SizedCostFunction<2, 7>
{
public:
    ProjectionCameraMapFactorPoseOnly(const Eigen::Vector3d &_map_point, const Eigen::Vector2d &_camera_point,
                           PinholeCamera *pPinholeCamera_);

    // calculate jacobian and residuals here
    //    -> residual should be Vector2d
    //    -> jacobian should be 1 * (2 * 7) -> ( one node * ( 2 residual elements * node's dimension 7 ))
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

private:
    Eigen::Vector3d map_point;
    Eigen::Vector2d camera_point;
    PinholeCamera *pPinholeCamera;
    double fx, fy, cx, cy;
};


// our number of residual should be 2 (the distance in 2D image space)
//     -> reproject the map point into image frame
// we optimize two nodes, the camera pose, whose dimension is 7
//                        and map point position, whose dimension is 3
class ProjectionCameraMapFactor : public ceres::SizedCostFunction<2, 7, 3>
{
public:
    ProjectionCameraMapFactor(const Eigen::Vector2d &_camera_point,
                           PinholeCamera *pPinholeCamera_);

    // calculate jacobian and residuals here
    //    -> residual should be Vector2d
    //    -> jacobian should be (2 * 7) and (2 * 3)
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    double EvaluateMine(double* camera_pose, double* map_point);

    //void check(double **parameters);

private:
    Eigen::Vector2d camera_point;
    PinholeCamera *pPinholeCamera;
    double fx, fy, cx, cy;

    double factor_map_point = 4000;
};





#endif // FACTOR_PNP_H
