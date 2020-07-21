#ifndef MYSLAM_BACKEND_PROBLEM_H
#define MYSLAM_BACKEND_PROBLEM_H

#include <unordered_map>
#include <map>
#include <memory>

#include "eigen_types.h"
#include "edge.h"
#include "backend/edge_reprojection.h"
#include "vertex.h"



typedef unsigned long ulong;

namespace myslam {
namespace backend {

typedef unsigned long ulong;
//    typedef std::unordered_map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
typedef std::map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
typedef std::unordered_map<unsigned long, std::shared_ptr<Edge>> HashEdge;
typedef std::unordered_multimap<unsigned long, std::shared_ptr<Edge>> HashVertexIdToEdge;

class Problem {
public:

    /**
     * 
     * seperate SLAM problem and general problem
     *
     *   -> if it is SLAM problem, pose and landmark are seperated and Hessian saved as sparse matrix
     *   -> and SLAM problem only accept certain vertex and edeg
     * 
     *   -> if general problem, Hessian is dense as default, unless some vertex be set marginalized
     * 
     */
    enum class ProblemType {
        SLAM_PROBLEM,
        GENERIC_PROBLEM
    };

    enum class SolverType {
        LM,
        LM_QUASI_NEWTON,
        DOG_LEG
    };


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Problem(ProblemType problemType);

    ~Problem();

    bool AddVertex(std::shared_ptr<Vertex> vertex);

    /**
     * remove a vertex
     * @param vertex_to_remove
     */
    bool RemoveVertex(std::shared_ptr<Vertex> vertex);

    bool AddEdge(std::shared_ptr<Edge> edge);

    bool RemoveEdge(std::shared_ptr<Edge> edge);

    /**
     * get the outliers judged by the optimzation
     * for the front end to delete it
     * @param outlier_edges
     */
    void GetOutlierEdges(std::vector<std::shared_ptr<Edge>> &outlier_edges);

    /**
     * solve the problem
     * @param iterations
     * @return
     */
    bool Solve(int iterations = 5);

    // marginalize a frame and the landmark "owned" by it (the landmarks whose host is this frame)
    bool Marginalize(std::shared_ptr<Vertex> frameVertex,
                     const std::vector<std::shared_ptr<Vertex>> &landmarkVerticies);

    bool Marginalize(const std::shared_ptr<Vertex> frameVertex);
    bool Marginalize(const std::vector<std::shared_ptr<Vertex> > frameVertex,int pose_dim);

    MatXX GetHessianPrior(){ return H_prior_;}
    VecX GetbPrior(){ return b_prior_;}
    VecX GetErrPrior(){ return err_prior_;}
    MatXX GetJtPrior(){ return Jt_prior_inv_;}

    void SetHessianPrior(const MatXX& H){H_prior_ = H;}
    void SetbPrior(const VecX& b){b_prior_ = b;}
    void SetErrPrior(const VecX& b){err_prior_ = b;}
    void SetJtPrior(const MatXX& J){Jt_prior_inv_ = J;}

    void ExtendHessiansPriorSize(int dim);

    //test compute prior
    void TestComputePrior();

private:

    /// Solve general probelm
    bool SolveGenericProblem(int iterations);

    /// Solve SLAM problem
    bool SolveSLAMProblem(int iterations);

    /// set the order of every index : ordering_index
    void SetOrdering();

    /// set ordering for new vertex in slam problem
    void AddOrderingSLAM(std::shared_ptr<Vertex> v);

    /// build the whole hessian matrix
    void MakeHessian();

    /// schur to solve SBA problem
    void SchurSBA();

    /// solve the linea problem
    void SolveLinearSystem();

    void SolveLinearSystemLM();

    /// update the states
    void UpdateStates();

    // sometimes the residual become larger after update -> bad update
    // require to roll back the former state
    void RollbackStates(); 

    /// compute and update the prior part
    void ComputePrior();

    /// whether a vertex is a pose
    bool IsPoseVertex(std::shared_ptr<Vertex> v);

    /// whether a vertex is a landmark
    bool IsLandmarkVertex(std::shared_ptr<Vertex> v);

    /// after adding a new vertex, the size changed of Hessian matrix
    void ResizePoseHessiansWhenAddingPose(std::shared_ptr<Vertex> v);

    /// check the state order
    bool CheckOrdering();

    void LogoutVectorSize();

    /// get the edges connect to a vertex
    std::vector<std::shared_ptr<Edge>> GetConnectedEdges(std::shared_ptr<Vertex> vertex);

    /// Levenberg

    /// initialize the Lambda parameter
    void ComputeLambdaInitLM();

    bool IsGoodStep();

    /// LM whether the lambda is good and how to update it
    bool IsGoodStepInLM();

    bool IsGoodStepInQN();

    /// PCG iteration solver
    VecX PCGSolver(const MatXX &A, const VecX &b, int maxIter);

public:

    double solvetime = 0;
    double hessiantime = 0;

    double shurtime = 0;
    int shurnumber = 0;

    bool verbose = true;

    // for save marginalized points
    bool ifRecordMap = true;
    std::vector<Vec3> marginalized_pts;
    std::vector<Vec3> current_pts;

private:
    ///////////// for LM QN solver ///////////////
    void MakeB();

    void SolveLinearSystemQN();

    // if use Hessian Quasi-Newton method
    bool currentMethodLM = true;         // if true : LM; if false : QN

    // an approximation B to the Hessian
    bool inited_B_ = false;
    MatXX B_;
    VecX b_old_;
    VecX b_new_;

    // trust region
    double trust_region_ = 100.00;
    double sqrt_rundoff = 1e-5;
    int count_LMQN = 0;

    bool IsGoodStepInQN_QN();
    bool IsGoodStepInQN_LM();

private:
    ////////////// for DOG LEG solver ///////////////
    //double trust_region_;  // defined above

    //double parameter_alpha;
    //double parameter_beta;
    
    //bool isfirst = true;

    double squared_norm_Jg;

    void SolveLinearSystemDL();

    bool IsGoodStepInDL();

    std::string logDL;

    //double b_norm;

private:

    int false_cnt = 0;

    double currentLambda_;
    double currentChi_;
    double stopThresholdLM_;    // LM threshold to quit
    double ni_;                 // the control the change of lambda

    ProblemType problemType_;
    //SolverType solverType_  = SolverType::DOG_LEG;
    SolverType solverType_  = SolverType::LM;

    /// the whole hessian
    MatXX Hessian_;
    VecX b_;
    //double b_norm;
    VecX delta_x_;

    /// prior part of the system
    MatXX H_prior_;
    VecX b_prior_;
    VecX b_prior_backup_;
    VecX err_prior_backup_;

    MatXX Jt_prior_inv_;
    VecX err_prior_;

    /// pose of SBA problem
    MatXX H_pp_schur_;
    VecX b_pp_schur_;

    // landmark(ll) and pose(pp) of hessian
    MatXX H_pp_;
    VecX b_pp_;
    MatXX H_ll_;
    VecX b_ll_;

    /// all vertices
    HashVertex verticies_;

    /// all edges
    HashEdge edges_;

    /// find edge by vertex id
    HashVertexIdToEdge vertexToEdge_;

    /// Ordering related
    ulong ordering_poses_ = 0;
    ulong ordering_landmarks_ = 0;
    ulong ordering_generic_ = 0;
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_pose_vertices_;        // order the pose vertex 
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_landmark_vertices_;    // order the landmark vertex

    // verticies need to marg. <Ordering_id_, Vertex>
    HashVertex verticies_marg_;

    bool bDebug = false;
    double t_hessian_cost_ = 0.0;
    double t_PCGsovle_cost_ = 0.0;
};

}
}

#endif
