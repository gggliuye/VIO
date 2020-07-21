#ifndef MYSLAM_BACKEND_EDGE_H
#define MYSLAM_BACKEND_EDGE_H

#include <memory>
#include <string>
#include "eigen_types.h"
#include <eigen3/Eigen/Dense>
#include "loss_function.h"

namespace myslam {
namespace backend {

class Vertex;

/**
 * edge
 * cost fcn = residual * infomation * residual
 * will be optimized by the backend process
 */
class Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * build fcn, will automaicly allocate space for jacobians
     * @param residual_dimension 
     * @param num_verticies 
     * @param verticies_types (optional)
     */
    explicit Edge(int residual_dimension, int num_verticies,
                  const std::vector<std::string> &verticies_types = std::vector<std::string>());

    virtual ~Edge();

    /// return the edge id
    unsigned long Id() const { return id_; }

    /**
     * add a vertex
     * @param vertex : vertex to add
     */
    bool AddVertex(std::shared_ptr<Vertex> vertex) {
        verticies_.emplace_back(vertex);
        return true;
    }

    /**
     * set some vertices
     * @param vertices : vertices with certain order
     * @return
     */
    bool SetVertex(const std::vector<std::shared_ptr<Vertex>> &vertices) {
        verticies_ = vertices;
        return true;
    }

    /// return the ith vertex
    std::shared_ptr<Vertex> GetVertex(int i) {
        return verticies_[i];
    }

    /// return all the vertices
    std::vector<std::shared_ptr<Vertex>> Verticies() const {
        return verticies_;
    }

    /// return the number of vertices connected
    size_t NumVertices() const { return verticies_.size(); }

    /// virtual fcn , return the edge type string
    virtual std::string TypeInfo() const = 0;

    /// virtual fcn, compute the residual 
    virtual void ComputeResidual() = 0;

    // whether the jacobian is already calculated, for FEJ (first estimate jacobian)
    bool jacobianSet = false;

    /// virtual fcn, conpute the jacobians
    virtual void ComputeJacobians() = 0;

    /// build only for reprojection edge, to get the world position of the corresponding point
    virtual Vec3 GetPointInWorld() { Vec3 tmp; return tmp;}

    virtual void CheckJacobian() { }

//    ///compute the influence to the hessian matrix
//    virtual void ComputeHessionFactor() = 0;

    /// compute the squared error, which will be timed by infomation matrix
    double Chi2() const;
    double RobustChi2() const;

    /// return the residual
    VecX Residual() const { return residual_; }

    /// return the residual size
    int ResidualDimension() const {return residual_dimension_;}

    /// return the jacobians
    std::vector<MatXX> Jacobians() const { return jacobians_; }

    /// set the infomation matrix
    void SetInformation(const MatXX &information) {
        information_ = information;
        // sqrt information
        sqrt_information_ = Eigen::LLT<MatXX>(information_).matrixL().transpose();
    }

    /// return the information matrix
    MatXX Information() const {
        return information_;
    }

    MatXX SqrtInformation() const {
        return sqrt_information_;
    }

    void SetLossFunction(LossFunction* ptr){ lossfunction_ = ptr; }
    LossFunction* GetLossFunction(){ return lossfunction_;}
    void RobustInfo(double& drho, MatXX& info) const;

    /// set observations
    void SetObservation(const VecX &observation) {
        observation_ = observation;
    }

    /// return observations
    VecX Observation() const { return observation_; }

    /// check if all the information necessary are properly set
    bool CheckValid();

    int OrderingId() const { return ordering_id_; }

    void SetOrderingId(int id) { ordering_id_ = id; };

protected:
    unsigned long id_;                               // edge id
    int ordering_id_;                                // edge id in problem
    std::vector<std::string> verticies_types_;       // info about all the edge for debug use
    std::vector<std::shared_ptr<Vertex>> verticies_; // corresponding vertices
    VecX residual_;                                  // residual
    std::vector<MatXX> jacobians_;                   // jacobian, dimension: #residual x #vertex[i]
    MatXX information_;                              // infoarmtion matrix (better call it hessian matrix)
    MatXX sqrt_information_;
    VecX observation_;                               // observations

    int residual_dimension_;
    LossFunction *lossfunction_;
};

}
}

#endif
