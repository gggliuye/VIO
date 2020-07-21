#ifndef G_FACTOR_H
#define G_FACTOR_H

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

float EvaluatePnP(double const *parameters,const cv::Point3f &map_point, const cv::Point2f &camera_point,
                  double &fx, double &fy, double &cx, double &cy)
{
    Eigen::Vector3d Pc(parameters[0], parameters[1], parameters[2]);
    Eigen::Quaterniond Qc(parameters[6], parameters[3], parameters[4], parameters[5]);
    Eigen::Vector3d map_point_eigen(map_point.x, map_point.y, map_point.z);

    Eigen::Vector3d Xc = Qc * map_point_eigen + Pc;
    double x = Xc(0);
    double y = Xc(1);
    double invz = 1.0 / Xc(2);

    float rx = - x * invz * fx - cx + camera_point.x;
    float ry = - y * invz * fy - cy + camera_point.y;
    return sqrt(rx*rx + ry*ry);
}

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

class PoseLocalParameterization : public ceres::LocalParameterization
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
class ProjectionCameraMapFactor : public ceres::SizedCostFunction<2, 7>
{
    Eigen::Vector3d map_point;
    Eigen::Vector2d camera_point;
    double fx, fy, cx, cy;

  public:
    ProjectionCameraMapFactor(const Eigen::Vector3d &_map_point, const Eigen::Vector2d &_camera_point, 
                           double _fx, double _fy, double _cx, double _cy): 
                     map_point(_map_point), camera_point(_camera_point), fx(_fx), fy(_fy), cx(_cx), cy(_cy){}

    // calculate jacobian and residuals here
    //    -> residual should be Vector2d
    //    -> jacobian should be 1 * (2 * 7) -> ( one node * ( 2 residual elements * node's dimension 7 ))
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Vector3d Pc(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qc(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Xc = Qc * map_point + Pc;
        double x = Xc(0);
        double y = Xc(1);
        double invz = 1.0 / Xc(2);
        double invz_2 = invz * invz;

        residuals[0] = - x * invz * fx - cx + camera_point(0);
        residuals[1] = - y * invz * fy - cy + camera_point(1);

        jacobians[0][0] = -invz *fx;
        jacobians[0][1] = 0;
        jacobians[0][2] = x*invz_2 *fx;
        jacobians[0][3] =  x*y*invz_2 *fx;
        jacobians[0][4] = -(1+(x*x*invz_2)) *fx;
        jacobians[0][5] = y*invz *fx;
        jacobians[0][6] = 0;

        jacobians[0][7] = 0;
        jacobians[0][8] = -invz *fy;
        jacobians[0][9] = y*invz_2 *fy;
        jacobians[0][10] = (1+y*y*invz_2) *fy;
        jacobians[0][11] = -x*y*invz_2 *fy;
        jacobians[0][12] = -x*invz *fy;
        jacobians[0][13] = 0; 

        return true;
    }

    void check(double **parameters)
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
        for (int k = 0; k < 6; k++)
        {
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
 
        std::cout << num_jacobian << std::endl;

    }
    

};

#include "ceres/rotation.h"
struct cost_function_define
{
    cost_function_define(cv::Point3f p1,cv::Point2f p2, double fx, double fy, double cx, double cy):
                    _p1(p1), _p2(p2), _fx(fx), _fy(fy), _cx(cx), _cy(cy)
    {
    }

    template<typename T>
    bool operator()(const T* const cere_r,const T* const cere_t, T* residual)const
    {
      const T _fxt = T(_fx);  
      const T _fyt = T(_fy);
      const T _cxt = T(_cx);
      const T _cyt = T(_cy);

      T p_1[3];
      T p_2[3];

      p_1[0] = T(_p1.x);
      p_1[1] = T(_p1.y);
      p_1[2] = T(_p1.z);

      ceres::AngleAxisRotatePoint(cere_r, p_1, p_2);

      p_2[0] = p_2[0] + cere_t[0];
      p_2[1] = p_2[1] + cere_t[1];
      p_2[2] = p_2[2] + cere_t[2];

      const T x = p_2[0] / p_2[2];
      const T y = p_2[1] / p_2[2];
      const T u = x * _fxt + _cxt;
      const T v = y * _fyt + _cyt;
    
      T u1 = T(_p2.x);
      T v1 = T(_p2.y);
 
      residual[0] = u1 - u;
      residual[1] = v1 - v;

      return true;
    }

     cv::Point3f _p1;
     cv::Point2f _p2;
     double _fx, _fy, _cx, _cy;
};



struct cost_function_pnp_quaternion
{
    cost_function_pnp_quaternion(cv::Point3f p1,cv::Point2f p2, double fx, double fy, double cx, double cy):
                    _p1(p1), _p2(p2), _fx(fx), _fy(fy), _cx(cx), _cy(cy)
    {
    }

    template<typename T>
    bool operator()(const T* const camera_pose, T* residual)const
    {
      const T _fxt = T(_fx);  
      const T _fyt = T(_fy);
      const T _cxt = T(_cx);
      const T _cyt = T(_cy);

      T p_1[3];
      T p_2[3];

      p_1[0] = T(_p1.x);
      p_1[1] = T(_p1.y);
      p_1[2] = T(_p1.z);

      T cam[4] = {camera_pose[6], camera_pose[3], camera_pose[4], camera_pose[5]};

      ceres::QuaternionRotatePoint(cam, p_1, p_2);

      p_2[0] = p_2[0] + camera_pose[0];
      p_2[1] = p_2[1] + camera_pose[1];
      p_2[2] = p_2[2] + camera_pose[2];

      const T x = p_2[0] / p_2[2];
      const T y = p_2[1] / p_2[2];
      const T u = x * _fxt + _cxt;
      const T v = y * _fyt + _cyt;
    
      T u1 = T(_p2.x);
      T v1 = T(_p2.y);
 
      residual[0] = u1 - u;
      residual[1] = v1 - v;

      return true;
    }

     cv::Point3f _p1;
     cv::Point2f _p2;
     double _fx, _fy, _cx, _cy;
};













#endif
