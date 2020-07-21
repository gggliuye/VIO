//
// Created by liuye on 19-8-22.
//

#ifndef SLAM_COURSE_EDGE_PRIOR_H
#define SLAM_COURSE_EDGE_PRIOR_H

#include <memory>
#include <string>

#include <Eigen/Dense>

#include "eigen_types.h"
#include "edge.h"


namespace myslam {
namespace backend {

/**
* EdgeMapFusion
* edge to fuse two map -> calculate the transformation between two map and the relative scale
* given the corresponding 3d points
* relative pose is the transform from i to j
*/

class EdgeMapFusion : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeMapFusion(Vec3 &pts_i, Vec3 &pts_j) :
            Edge(3, 2, std::vector<std::string>{"VertexPose", "VertexScale"}),
            pts_i_(pts_i), pts_j_(pts_j){}

    /// return the edge type
    virtual std::string TypeInfo() const override { return "EdgeMapFusion"; }

    virtual void ComputeResidual() override;

    virtual void ComputeJacobians() override;

    virtual void CheckJacobian() override;

private:
    Vec3 pts_i_;
    Vec3 pts_j_;
};

}
}

#endif
