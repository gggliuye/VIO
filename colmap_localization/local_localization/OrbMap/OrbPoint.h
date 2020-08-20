#ifndef ORB_POINT_UTOPA_H_
#define ORB_POINT_UTOPA_H_

#include "configure.h"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace BASTIAN
{

class OrbPoint
{
public:
    OrbPoint(Eigen::Vector3d mPose_){
        mPose = mPose_;
    }

    Eigen::Vector3d mPose;

};

} // namespace BASTIAN

#endif
