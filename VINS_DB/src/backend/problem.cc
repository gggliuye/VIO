#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include "backend/problem.h"
#include "utility/tic_toc.h"

#ifdef USE_OPENMP

#include <omp.h>

#endif

using namespace std;

// define the format you want, you only need one instance of this...
const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

void writeToCSVfile(std::string name, Eigen::MatrixXd matrix) {
    std::ofstream f(name.c_str());
    f << matrix.format(CSVFormat);
}

namespace myslam {
namespace backend {
void Problem::LogoutVectorSize() {
    // LOG(INFO) <<
    //           "1 problem::LogoutVectorSize verticies_:" << verticies_.size() <<
    //           " edges:" << edges_.size();
}

Problem::Problem(ProblemType problemType) :
    problemType_(problemType) {
    LogoutVectorSize();
    verticies_marg_.clear();

}

Problem::~Problem() {
//    std::cout << "Problem IS Deleted"<<std::endl;
    global_vertex_id = 0;
}

bool Problem::AddVertex(std::shared_ptr<Vertex> vertex) {
    if (verticies_.find(vertex->Id()) != verticies_.end()) {
        // LOG(WARNING) << "Vertex " << vertex->Id() << " has been added before";
        return false;
    } else {
        verticies_.insert(pair<unsigned long, shared_ptr<Vertex>>(vertex->Id(), vertex));
    }

    if (problemType_ == ProblemType::SLAM_PROBLEM) {
        if (IsPoseVertex(vertex)) {
            ResizePoseHessiansWhenAddingPose(vertex);
        }
    }
    return true;
}

void Problem::AddOrderingSLAM(std::shared_ptr<myslam::backend::Vertex> v) {
    if (IsPoseVertex(v)) {
        v->SetOrderingId(ordering_poses_);
        idx_pose_vertices_.insert(pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
        ordering_poses_ += v->LocalDimension();
    } else if (IsLandmarkVertex(v)) {
        v->SetOrderingId(ordering_landmarks_);
        ordering_landmarks_ += v->LocalDimension();
        idx_landmark_vertices_.insert(pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
    }
}

void Problem::ResizePoseHessiansWhenAddingPose(shared_ptr<Vertex> v) {

    int size = H_prior_.rows() + v->LocalDimension();
    H_prior_.conservativeResize(size, size);
    b_prior_.conservativeResize(size);

    b_prior_.tail(v->LocalDimension()).setZero();
    H_prior_.rightCols(v->LocalDimension()).setZero();
    H_prior_.bottomRows(v->LocalDimension()).setZero();

}
void Problem::ExtendHessiansPriorSize(int dim)
{
    int size = H_prior_.rows() + dim;
    H_prior_.conservativeResize(size, size);
    b_prior_.conservativeResize(size);

    b_prior_.tail(dim).setZero();
    H_prior_.rightCols(dim).setZero();
    H_prior_.bottomRows(dim).setZero();
}

bool Problem::IsPoseVertex(std::shared_ptr<myslam::backend::Vertex> v) {
    string type = v->TypeInfo();
    return type == string("VertexPose") ||
            type == string("VertexSpeedBias");
}

bool Problem::IsLandmarkVertex(std::shared_ptr<myslam::backend::Vertex> v) {
    string type = v->TypeInfo();
    return type == string("VertexPointXYZ") ||
           type == string("VertexInverseDepth");
}

bool Problem::AddEdge(shared_ptr<Edge> edge) {
    if (edges_.find(edge->Id()) == edges_.end()) {
        edges_.insert(pair<ulong, std::shared_ptr<Edge>>(edge->Id(), edge));
    } else {
        // LOG(WARNING) << "Edge " << edge->Id() << " has been added before!";
        return false;
    }

    for (auto &vertex: edge->Verticies()) {
        vertexToEdge_.insert(pair<ulong, shared_ptr<Edge>>(vertex->Id(), edge));
    }
    return true;
}

vector<shared_ptr<Edge>> Problem::GetConnectedEdges(std::shared_ptr<Vertex> vertex) {

    vector<shared_ptr<Edge>> edges;
    auto range = vertexToEdge_.equal_range(vertex->Id());
    for (auto iter = range.first; iter != range.second; ++iter) {

        // the edge needs to be existm, and not be removed
        if (edges_.find(iter->second->Id()) == edges_.end())
            continue;

        edges.emplace_back(iter->second);
    }
    return edges;
}

bool Problem::RemoveVertex(std::shared_ptr<Vertex> vertex) {
    //check if the vertex is in map_verticies_
    if (verticies_.find(vertex->Id()) == verticies_.end()) {
        // LOG(WARNING) << "The vertex " << vertex->Id() << " is not in the problem!" << endl;
        return false;
    }

    // delete the edges connected with the vertex
    vector<shared_ptr<Edge>> remove_edges = GetConnectedEdges(vertex);

    for (size_t i = 0; i < remove_edges.size(); i++) {
        RemoveEdge(remove_edges[i]);
    }

    if (IsPoseVertex(vertex))
        idx_pose_vertices_.erase(vertex->Id());
    else
        idx_landmark_vertices_.erase(vertex->Id());

    vertex->SetOrderingId(-1);      // used to debug
    verticies_.erase(vertex->Id());
    vertexToEdge_.erase(vertex->Id());

    return true;
}

bool Problem::RemoveEdge(std::shared_ptr<Edge> edge) {
    //check if the edge is in map_edges_
    if (edges_.find(edge->Id()) == edges_.end()) {
        // LOG(WARNING) << "The edge " << edge->Id() << " is not in the problem!" << endl;
        return false;
    }

    edges_.erase(edge->Id());
    return true;
}

bool Problem::Solve(int iterations) {

    if (edges_.size() == 0 || verticies_.size() == 0) {
        std::cerr << "\nCannot solve problem without edges or verticies" << std::endl;
        return false;
    }

    TicToc t_solve;
    // compute the dimensions of the system, for later build hessian
    SetOrdering();
    // make the hessian from all the edges
    MakeHessian();
    // LM initizalization
    ComputeLambdaInitLM();

    // LM main solve iteratin
    bool stop = false;
    int iter = 0;
    double last_chi_ = 1e20;
    while (!stop && (iter < iterations)) {
        if(verbose && solverType_ == SolverType::LM)
            std::cout << "iter: " << iter << " , chi = " << currentChi_ << " , Lambda = " << currentLambda_ << std::endl;
        if(verbose && solverType_ == SolverType::DOG_LEG)
            std::cout << "iter: " << iter << " , chi = " << currentChi_ << " , trust region radius = " << trust_region_ <<  " , method " << logDL << std::endl;
        bool oneStepSuccess = false;
        false_cnt = 0;
        while (!oneStepSuccess && false_cnt < 10)  // keep iteration
        {
            // solve the linear system
            SolveLinearSystem();
            // update the states
            UpdateStates();
            // evaluate the current step, set the next lambda and compute chi2
            oneStepSuccess = IsGoodStep();
        }
        iter++;

        // if the current residual improvement too small  

        if(last_chi_ - currentChi_ < 1e-5)
        {
            if(verbose)
                std::cout << " [SOLVER STOP] the residual reduce too less (< 1e-5)" << std::endl;
            stop = true;
        }
/*
        //std::cout << "  delta_x_.norm() : " << delta_x_.norm()<< std::endl;
        if(delta_x_.norm() < 1e-10)
        {
            if(verbose)
                std::cout << " [LM STOP] the delta x too less (< 1e-10)" << std::endl;
            stop = true;
        }
*/
        last_chi_ = currentChi_;
    }

    if(verbose){
        std::cout << " [SOLVER FINISH]  problem solve cost: " << t_solve.toc() << " ms" << std::endl;
        std::cout << " [SOLVER FINISH]  makeHessian cost: " << t_hessian_cost_ << " ms" << std::endl;
    }
    
    solvetime = t_solve.toc();
    hessiantime = t_hessian_cost_;
    //runtimefile << t_solve.toc() << " " << t_hessian_cost_ << std::endl;

    t_hessian_cost_ = 0.;
    return true;
}

bool Problem::SolveGenericProblem(int iterations) {
    return true;
}

void Problem::SetOrdering() {

    // reset the ordering parameters
    ordering_poses_ = 0;
    ordering_generic_ = 0;
    ordering_landmarks_ = 0;

    // Note:: verticies_ is a map , ordered by its ids
    for (auto vertex: verticies_) {
        ordering_generic_ += vertex.second->LocalDimension();  // the dimension of oprimization
        // if it is a SLAM problem , need the compute dimension of poses and landmarks, and later ordering them
        if (problemType_ == ProblemType::SLAM_PROBLEM)    
        {
            AddOrderingSLAM(vertex.second);
        }
    }
    if (problemType_ == ProblemType::SLAM_PROBLEM) {
        // add the landmark order with the number of pose -> to keep the landmark ordered after poses
        ulong all_pose_dimension = ordering_poses_;
        for (auto landmarkVertex : idx_landmark_vertices_) {
            landmarkVertex.second->SetOrderingId(
                landmarkVertex.second->OrderingId() + all_pose_dimension
            );
        }
    }
    //CHECK_EQ(CheckOrdering(), true);
}

bool Problem::CheckOrdering() {
    if (problemType_ == ProblemType::SLAM_PROBLEM) {
        int current_ordering = 0;
        for (auto v: idx_pose_vertices_) {
            assert(v.second->OrderingId() == current_ordering);
            current_ordering += v.second->LocalDimension();
        }

        for (auto v: idx_landmark_vertices_) {
            assert(v.second->OrderingId() == current_ordering);
            current_ordering += v.second->LocalDimension();
        }
    }
    return true;
}

void Problem::MakeB(){
    ulong size = ordering_generic_;
    if(!inited_B_){
        B_.resize(size,size);
        B_ << MatXX::Identity(size, size);
        inited_B_ = true;
    }

    VecX y(VecX::Zero(size));  
    y = Hessian_ * delta_x_;

    double hty = (delta_x_.transpose() * y);

    if(hty > 0){
        auto v = B_ * delta_x_;
        double htv = delta_x_.transpose() * v;
        B_ = B_ + y * y.transpose() / hty - v * v.transpose() / htv;
    }
}

void Problem::MakeHessian() {
    TicToc t_h;
    // build the large Hessian matrix
    ulong size = ordering_generic_;
    MatXX H(MatXX::Zero(size, size));
    VecX b(VecX::Zero(size));

    #pragma omp parallel 
    #pragma omp single
    {
    for (auto &edge: edges_) {
        #pragma omp task firstprivate(edge)
        {
        edge.second->ComputeResidual();
        edge.second->ComputeJacobians();

        auto jacobians = edge.second->Jacobians();
        auto verticies = edge.second->Verticies();
        assert(jacobians.size() == verticies.size());
        for (size_t i = 0; i < verticies.size(); ++i) {
            auto v_i = verticies[i];
            // if the vertex is fixed -> should not change its value -> set its jacobian to be zeros
            if (v_i->IsFixed()) continue;

            auto jacobian_i = jacobians[i];
            ulong index_i = v_i->OrderingId();
            ulong dim_i = v_i->LocalDimension();

            // set the robust cost function
            double drho;
            MatXX robustInfo(edge.second->Information().rows(),edge.second->Information().cols());
            edge.second->RobustInfo(drho,robustInfo);

            MatXX JtW = jacobian_i.transpose() * robustInfo;
            for (size_t j = i; j < verticies.size(); ++j) {
                auto v_j = verticies[j];

                if (v_j->IsFixed()) continue;

                auto jacobian_j = jacobians[j];
                ulong index_j = v_j->OrderingId();
                ulong dim_j = v_j->LocalDimension();

                assert(v_j->OrderingId() != -1);
                MatXX hessian = JtW * jacobian_j;

                // acculumate all together
                #pragma omp critical
                H.block(index_i, index_j, dim_i, dim_j).noalias() += hessian;
                if (j != i) {
                    #pragma omp critical
                    H.block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();
                }
            }
            #pragma omp critical
            b.segment(index_i, dim_i).noalias() -= drho * jacobian_i.transpose()* edge.second->Information() * edge.second->Residual();
        }

        } // task

        //#pragma omp taskwait
    }

    } // openmp single

    b_old_ = b_;
    Hessian_ = H;
    b_ = b;
    t_hessian_cost_ += t_h.toc();

    if(H_prior_.rows() > 0)
    {
        MatXX H_prior_tmp = H_prior_;
        VecX b_prior_tmp = b_prior_;

        /// for all the pose vertices, set its prior dimension to be zero, and fix its external parameters 
        /// landmark has no prior
        for (auto vertex: verticies_) {
            if (IsPoseVertex(vertex.second) && vertex.second->IsFixed() ) {
                int idx = vertex.second->OrderingId();
                int dim = vertex.second->LocalDimension();
                H_prior_tmp.block(idx,0, dim, H_prior_tmp.cols()).setZero();
                H_prior_tmp.block(0,idx, H_prior_tmp.rows(), dim).setZero();
                b_prior_tmp.segment(idx,dim).setZero();
                // std::cout << " fixed prior, set the Hprior and bprior part to zero, idx: "<<idx <<" dim: "<<dim<<std::endl;
            } 
        }
        Hessian_.topLeftCorner(ordering_poses_, ordering_poses_) += H_prior_tmp;
        b_.head(ordering_poses_) += b_prior_tmp;
    }

    if (solverType_ != SolverType::DOG_LEG){
        return;
    }

    squared_norm_Jg = 0;

    for (auto &edge: edges_) {
        int dimResidual = edge.second->ResidualDimension();
        VecX tmp;
        tmp.resize(dimResidual);
        tmp.setZero();

        auto jacobians = edge.second->Jacobians();
        auto verticies = edge.second->Verticies();
        for (size_t i = 0; i < verticies.size(); ++i) {
            auto v_i = verticies[i];
            if (v_i->IsFixed()) continue;

            auto jacobian_i = jacobians[i];
            ulong index_i = v_i->OrderingId();
            ulong dim_i = v_i->LocalDimension();
            
            tmp += jacobian_i * (b_.segment(index_i, dim_i));

            //std::cout << "lalalalaalala" << std::endl;
            //std::cout << jacobian_i << std::endl;
            //std::cout << b_.segment(index_i, dim_i) << std::endl;
        }
        squared_norm_Jg += tmp.squaredNorm();
    }

    //std::cout << "     squared_norm_Jg : " << squared_norm_Jg << std::endl;
    //std::cout << "     alpha           : " << b_.squaredNorm() / squared_norm_Jg << std::endl;
    
    //delta_x_ = VecX::Zero(size);  // initial delta_x = 0_n;
}

void Problem::SolveLinearSystem()
{
    if(solverType_ == SolverType::LM){
        SolveLinearSystemLM();
    }
    if(solverType_ == SolverType::LM_QUASI_NEWTON){
        if(currentMethodLM){
            SolveLinearSystemLM();
        }
        else{
            SolveLinearSystemQN();
        }
    }
    else if (solverType_ == SolverType::DOG_LEG){
        SolveLinearSystemDL();
    }
    else{
        //  if not found method, use LM by default
        SolveLinearSystemLM();
    }

}


/*
 *   -- Levenberg-Marquardt step --
 * Solve Hessian matrix and calculate the state update 
 *
 * Solve Hx = b, we can use PCG iterative method or use sparse Cholesky
 */
void Problem::SolveLinearSystemLM() {
    ulong size = ordering_generic_;
    delta_x_ = VecX::Zero(size);  // initial delta_x = 0_n;

    if (problemType_ == ProblemType::GENERIC_PROBLEM) {
        // PCG solver
        MatXX H = Hessian_;
        for (int i = 0; i < Hessian_.cols(); ++i) {
            H(i, i) += currentLambda_;
        }
        // delta_x_ = PCGSolver(H, b_, H.rows() * 2);
        delta_x_ = H.ldlt().solve(b_);

    } else {

        TicToc t_Hmminv;
        // step1: schur marginalization --> Hpp, bpp
        int reserve_size = ordering_poses_;
        int marg_size = ordering_landmarks_;
        MatXX Hmm = Hessian_.block(reserve_size, reserve_size, marg_size, marg_size);
        MatXX Hpm = Hessian_.block(0, reserve_size, reserve_size, marg_size);
        MatXX Hmp = Hessian_.block(reserve_size, 0, marg_size, reserve_size);
        VecX bpp = b_.segment(0, reserve_size);
        VecX bmm = b_.segment(reserve_size, marg_size);

        // Hmm 是对角线矩阵，它的求逆可以直接为对角线块分别求逆，如果是逆深度，对角线块为1维的，则直接为对角线的倒数，这里可以加速
        MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
        // TODO:: use openMP
        for (auto landmarkVertex : idx_landmark_vertices_) {
            int idx = landmarkVertex.second->OrderingId() - reserve_size;
            int size = landmarkVertex.second->LocalDimension();
            Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
        }

        MatXX tempH = Hpm * Hmm_inv;
        H_pp_schur_ = Hessian_.block(0, 0, ordering_poses_, ordering_poses_) - tempH * Hmp;
        b_pp_schur_ = bpp - tempH * bmm;

        // step2: solve Hpp * delta_x = bpp
        VecX delta_x_pp(VecX::Zero(reserve_size));

        for (ulong i = 0; i < ordering_poses_; ++i) {
            H_pp_schur_(i, i) += currentLambda_;              // LM Method
        }

        // TicToc t_linearsolver;
        delta_x_pp =  H_pp_schur_.ldlt().solve(b_pp_schur_);//  SVec.asDiagonal() * svd.matrixV() * Ub;    
        // std::cout << " Linear Solver Time Cost: " << t_linearsolver.toc() << std::endl;

        //Eigen::JacobiSVD<Eigen::MatrixXd> svd(H_pp_schur_, Eigen::ComputeThinU | Eigen::ComputeThinV);
        //delta_x_pp = svd.solve(b_pp_schur_);


        delta_x_.head(reserve_size) = delta_x_pp;

        // step3: solve Hmm * delta_x = bmm - Hmp * delta_x_pp;
        VecX delta_x_ll(marg_size);
        delta_x_ll = Hmm_inv * (bmm - Hmp * delta_x_pp);
        delta_x_.tail(marg_size) = delta_x_ll;

//        std::cout << "schur time cost: "<< t_Hmminv.toc()<<std::endl;
        shurtime += t_Hmminv.toc();
        shurnumber += 1;
    }
}


/*
 *   -- DOG LEG step --
 * 
 */
void Problem::SolveLinearSystemDL() {

    ulong size = ordering_generic_;
    VecX delta_x_GN = VecX::Zero(size);
    VecX delta_x_SD = VecX::Zero(size);
    delta_x_SD = b_;

    if (problemType_ == ProblemType::GENERIC_PROBLEM) {
        // PCG solver
        MatXX H = Hessian_;
        delta_x_GN = H.ldlt().solve(b_);
    } else {

        TicToc t_Hmminv;

        // step1: schur marginalization --> Hpp, bpp
        int reserve_size = ordering_poses_;
        int marg_size = ordering_landmarks_;
        MatXX Hmm = Hessian_.block(reserve_size, reserve_size, marg_size, marg_size);
        MatXX Hpm = Hessian_.block(0, reserve_size, reserve_size, marg_size);
        MatXX Hmp = Hessian_.block(reserve_size, 0, marg_size, reserve_size);
        VecX bpp = b_.segment(0, reserve_size);
        VecX bmm = b_.segment(reserve_size, marg_size);

        // Hmm 是对角线矩阵，它的求逆可以直接为对角线块分别求逆，如果是逆深度，对角线块为1维的，则直接为对角线的倒数，这里可以加速
        MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));

        #pragma omp parallel 
        #pragma omp single
        for (auto landmarkVertex : idx_landmark_vertices_) {
            #pragma omp task firstprivate(landmarkVertex)
            {
            int idx = landmarkVertex.second->OrderingId() - reserve_size;
            int size = landmarkVertex.second->LocalDimension();
            // #pragma omp critical // the memory will not conflit
            Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
            }
            //#pragma omp taskwait // tasks have no order
        }

        MatXX tempH = Hpm * Hmm_inv;
        H_pp_schur_ = Hessian_.block(0, 0, ordering_poses_, ordering_poses_) - tempH * Hmp;
        b_pp_schur_ = bpp - tempH * bmm;

        // step2: solve Hpp * delta_x = bpp
        VecX delta_x_pp(VecX::Zero(reserve_size));

        // TicToc t_linearsolver;
        delta_x_pp =  H_pp_schur_.ldlt().solve(b_pp_schur_);//  SVec.asDiagonal() * svd.matrixV() * Ub;    
        delta_x_GN.head(reserve_size) = delta_x_pp;
        // std::cout << " Linear Solver Time Cost: " << t_linearsolver.toc() << std::endl;

        // step3: solve Hmm * delta_x = bmm - Hmp * delta_x_pp;
        VecX delta_x_ll(marg_size);
        delta_x_ll = Hmm_inv * (bmm - Hmp * delta_x_pp);
        delta_x_GN.tail(marg_size) = delta_x_ll;

        //std::cout << "schur time cost: "<< t_Hmminv.toc() <<std::endl;

        shurtime += t_Hmminv.toc();
        shurnumber += 1;
    }

    // calculate parameter alpha
    double b_squarednorm = b_.squaredNorm();  // it is equal to delta_x_SD_squarednorm
    double alpha_ = b_squarednorm / squared_norm_Jg;
    double delta_x_SD_norm = delta_x_SD.norm();

    //b_norm = delta_x_SD_norm

    logDL = "";
    if(delta_x_GN.norm() <= trust_region_){
        // use Gauss Newton method
        delta_x_ = delta_x_GN;
        logDL = "use GN";
    }
    else if(alpha_ * delta_x_SD_norm >= trust_region_ ){
        // use steepest descent method
        delta_x_ = ( trust_region_ / delta_x_SD_norm) * delta_x_SD;
        logDL = "use SD";
    }
    else{
        // h_gl = alpha * h_sd + beta * (h_gn - alpha * h_sd)
        // a = alpha * delta_x_SD
        // b = delta_x_GN
        // c = a' (b - a)
        double b_dot_a = alpha_ * delta_x_SD.dot(delta_x_GN);
        double a_squared_norm = pow(alpha_ * delta_x_SD_norm, 2);
        double b_minus_a_squared_norm = a_squared_norm - 2 * b_dot_a + delta_x_GN.squaredNorm();

        double squared_trust_region_ = trust_region_ * trust_region_;

        double c = b_dot_a - a_squared_norm;
        double d = sqrt(c * c + b_minus_a_squared_norm *
                        (squared_trust_region_ - a_squared_norm));

        double beta_ = (c <= 0) ? (d - c) /  b_minus_a_squared_norm : (squared_trust_region_ - a_squared_norm) / (d + c);
        delta_x_ = (alpha_ * (1.0 - beta_)) * delta_x_SD + beta_ * delta_x_GN;
        logDL = "use mixed";
    }
}


/*
 *   -- Quasi-Newton step --
 *
 */
void Problem::SolveLinearSystemQN() {
   
    if (problemType_ == ProblemType::GENERIC_PROBLEM) {
        MatXX H = B_;
        delta_x_ = H.ldlt().solve(b_);

    } else {
        // step1: schur marginalization --> Hpp, bpp
        int reserve_size = ordering_poses_;
        int marg_size = ordering_landmarks_;
        MatXX Hmm = B_.block(reserve_size, reserve_size, marg_size, marg_size);
        MatXX Hpm = B_.block(0, reserve_size, reserve_size, marg_size);
        MatXX Hmp = B_.block(reserve_size, 0, marg_size, reserve_size);
        VecX bpp = b_.segment(0, reserve_size);
        VecX bmm = b_.segment(reserve_size, marg_size);

        MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
        // TODO:: use openMP
        for (auto landmarkVertex : idx_landmark_vertices_) {
            int idx = landmarkVertex.second->OrderingId() - reserve_size;
            int size = landmarkVertex.second->LocalDimension();
            Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
        }

        MatXX tempH = Hpm * Hmm_inv;
        H_pp_schur_ = B_.block(0, 0, ordering_poses_, ordering_poses_) - tempH * Hmp;
        b_pp_schur_ = bpp - tempH * bmm;

        // step2: solve Hpp * delta_x = bpp
        VecX delta_x_pp(VecX::Zero(reserve_size));
        delta_x_pp =  H_pp_schur_.ldlt().solve(b_pp_schur_);//  SVec.asDiagonal() * svd.matrixV() * Ub;    
        delta_x_.head(reserve_size) = delta_x_pp;

        // step3: solve Hmm * delta_x = bmm - Hmp * delta_x_pp;
        VecX delta_x_ll(marg_size);
        delta_x_ll = Hmm_inv * (bmm - Hmp * delta_x_pp);
        delta_x_.tail(marg_size) = delta_x_ll;
    }
    
    double delta_x_norm = delta_x_.norm();
    if(delta_x_norm > trust_region_){
        delta_x_ = (trust_region_ / delta_x_norm) * delta_x_;
    }
}

void Problem::UpdateStates() {

    // update vertex
    for (auto vertex: verticies_) {
        vertex.second->BackUpParameters();    // backup last values
        ulong idx = vertex.second->OrderingId();
        ulong dim = vertex.second->LocalDimension();
        VecX delta = delta_x_.segment(idx, dim);
        vertex.second->Plus(delta);
    }
    // update prior
    if (err_prior_.rows() > 0) {
        // BACK UP b_prior_
        b_prior_backup_ = b_prior_;
        err_prior_backup_ = err_prior_;

        /// update with first order Taylor, b' = b + \frac{\delta b}{\delta x} * \delta x
        /// \delta x = Computes the linearized deviation from the references (linearization points)
        b_prior_ -= H_prior_ * delta_x_.head(ordering_poses_);       // update the error_prior
        err_prior_ = -Jt_prior_inv_ * b_prior_.head(ordering_poses_ - 15);
//        std::cout << "                : "<< b_prior_.norm()<<" " <<err_prior_.norm()<< std::endl;
//        std::cout << "     delta_x_ ex: "<< delta_x_.head(6).norm() << std::endl;
    }

}

void Problem::RollbackStates() {
    // update vertex
    for (auto vertex: verticies_) {
        vertex.second->RollBackParameters();
    }
    // Roll back prior_
    if (err_prior_.rows() > 0) {
        b_prior_ = b_prior_backup_;
        err_prior_ = err_prior_backup_;
    }
}

/// LM
void Problem::ComputeLambdaInitLM() {
    ni_ = 2.;
    currentLambda_ = -1.;
    currentChi_ = 0.0;

    for (auto edge: edges_) {
        currentChi_ += edge.second->RobustChi2();
    }
    if (err_prior_.rows() > 0)
        currentChi_ += err_prior_.norm();
    currentChi_ *= 0.5;

    double maxDiagonal = 0;
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    for (ulong i = 0; i < size; ++i) {
        maxDiagonal = std::max(fabs(Hessian_(i, i)), maxDiagonal);
    }

    maxDiagonal = std::min(5e10, maxDiagonal);
    double tau = 1e-5;  // 1e-5
    currentLambda_ = tau * maxDiagonal;

}

bool Problem::IsGoodStep(){
    bool oneStepSuccess = false;
    if(solverType_ == SolverType::LM){
        oneStepSuccess = IsGoodStepInLM();
        // if success
        if (oneStepSuccess) {
        // make the new hessian matrix
            MakeHessian();
            false_cnt = 0;
        } else {
            false_cnt ++;
            // it was a bad update roll back to previous state
            RollbackStates(); 
        }
    }
    else if(solverType_ == SolverType::LM_QUASI_NEWTON){

        oneStepSuccess = IsGoodStepInQN();

        if (oneStepSuccess) { 
            MakeHessian();
            false_cnt = 0;
        } else {
            false_cnt ++;
            RollbackStates(); 
        }
        MakeB();
    }
    else if(solverType_ == SolverType::DOG_LEG){

        oneStepSuccess = IsGoodStepInDL();
        // if success
        if (oneStepSuccess) {
            MakeHessian();
            false_cnt = 0;
        } else {
            false_cnt ++;
            RollbackStates(); 
        }
    }
    return oneStepSuccess;

}

bool Problem::IsGoodStepInDL(){
    double scale = 0;

    scale = 0.5* delta_x_.transpose() * (currentLambda_ * delta_x_ + b_);
    scale += 1e-8;    // make sure it's non-zero :)

    // recompute residuals after update state
    double tempChi = 0.0;
    for (auto edge: edges_) {
        edge.second->ComputeResidual();
        tempChi += edge.second->RobustChi2();
    }
    if (err_prior_.size() > 0)
        tempChi += err_prior_.norm();
    tempChi *= 0.5;          // 1/2 * err^2

    double rho = (currentChi_ - tempChi) / scale;

    if(rho > 0  && isfinite(tempChi)){
        if(rho > 0.75){
            trust_region_ = std::max(trust_region_, 3*delta_x_.norm());
        }
        currentChi_ = tempChi;
        return true;
    }
    else{
        if(rho < 0.25){
            trust_region_ = trust_region_/2;
        }
        return false;
    }
}

bool Problem::IsGoodStepInLM() {
    double scale = 0;

    scale = 0.5* delta_x_.transpose() * (currentLambda_ * delta_x_ + b_);
    scale += 1e-8;    // make sure it's non-zero :)

    // recompute residuals after update state
    double tempChi = 0.0;
    for (auto edge: edges_) {
        edge.second->ComputeResidual();
        tempChi += edge.second->RobustChi2();
    }
    if (err_prior_.size() > 0)
        tempChi += err_prior_.norm();
    tempChi *= 0.5;          // 1/2 * err^2

    double rho = (currentChi_ - tempChi) / scale;
    if (rho > 0 && isfinite(tempChi))   // last step was good
    {
        double alpha = 1. - pow((2 * rho - 1), 3);
        alpha = std::min(alpha, 2. / 3.);
        double scaleFactor = (std::max)(1. / 3., alpha);
        currentLambda_ *= scaleFactor;
        ni_ = 2;
        currentChi_ = tempChi;
        return true;
    } else {
        currentLambda_ *= ni_;
        ni_ *= 2;
        return false;
    }
}

bool Problem::IsGoodStepInQN(){

    bool isgood = false;

    if(currentMethodLM){
        isgood = IsGoodStepInQN_LM();
    }
    else{
        isgood = IsGoodStepInQN_QN();
    }

    return isgood;
}

bool Problem::IsGoodStepInQN_LM(){
    double scale = 0;

    scale = 0.5* delta_x_.transpose() * (currentLambda_ * delta_x_ + b_);
    scale += 1e-6;    // make sure it's non-zero :)

    // recompute residuals after update state
    double tempChi = 0.0;
    for (auto edge: edges_) {
        edge.second->ComputeResidual();
        tempChi += edge.second->RobustChi2();
    }
    if (err_prior_.size() > 0)
        tempChi += err_prior_.norm();
    tempChi *= 0.5;          // 1/2 * err^2

    double rho = (currentChi_ - tempChi) / scale;

    if (rho > 0 && isfinite(tempChi))   // last step was good
    {
        if(b_old_.norm() < 0.02 * tempChi){
            count_LMQN += 1;
            if(count_LMQN >=3){
                 currentMethodLM = false;
                 // TODO
                 trust_region_ = (std::max)(1.5*1e-5*1, delta_x_.norm()/5);
            }
        }
        else{
            count_LMQN = 0;
        }
        double alpha = 1. - pow((2 * rho - 1), 3);
        alpha = std::min(alpha, 2. / 3.);
        double scaleFactor = (std::max)(1. / 3., alpha);
        currentLambda_ *= scaleFactor;
        ni_ = 2;
        currentChi_ = tempChi;

        return true;
    } else {
        currentLambda_ *= ni_;
        ni_ *= 2;
        count_LMQN = 0;
        return false;
    }


}

bool Problem::IsGoodStepInQN_QN(){
    // recompute residuals after update state -> F(x_new)
    double tempChi = 0.0;

    for (auto edge: edges_) {
        edge.second->ComputeResidual();
        tempChi += edge.second->RobustChi2();
    }
    if (err_prior_.size() > 0)
        tempChi += err_prior_.norm();
    tempChi *= 0.5;

    if(b_.norm() >= b_old_.norm()){
        currentMethodLM = true;
    }

    if(currentChi_ > tempChi && isfinite(tempChi)){
        return true;
    }
    else if((1+sqrt_rundoff)*currentChi_ > tempChi && b_.norm() < b_old_.norm() && isfinite(tempChi)){
        return true;
    }
    else{
        return false;
    }
}

/** @brief conjugate gradient with perconditioning
 *
 *  the jacobi PCG method
 *
 */
VecX Problem::PCGSolver(const MatXX &A, const VecX &b, int maxIter = -1) {
    assert(A.rows() == A.cols() && "PCG solver ERROR: A is not a square matrix");
    int rows = b.rows();
    int n = maxIter < 0 ? rows : maxIter;
    VecX x(VecX::Zero(rows));
    MatXX M_inv = A.diagonal().asDiagonal().inverse();
    VecX r0(b);  // initial r = b - A*0 = b
    VecX z0 = M_inv * r0;
    VecX p(z0);
    VecX w = A * p;
    double r0z0 = r0.dot(z0);
    double alpha = r0z0 / p.dot(w);
    VecX r1 = r0 - alpha * w;
    int i = 0;
    double threshold = 1e-6 * r0.norm();
    while (r1.norm() > threshold && i < n) {
        i++;
        VecX z1 = M_inv * r1;
        double r1z1 = r1.dot(z1);
        double belta = r1z1 / r0z0;
        z0 = z1;
        r0z0 = r1z1;
        r0 = r1;
        p = belta * p + z1;
        w = A * p;
        alpha = r1z1 / p.dot(w);
        x += alpha * p;
        r1 -= alpha * w;
    }
    return x;
}

/*
* marginalize all the edges connected with the frame (imu factors and the projection factors)
* 
* If we want to keep a landmark which is connected with the frame, we should first delete this edge, 
* otherwise the Hessian will become denser, which will slow down the process. 
*/
bool Problem::Marginalize(const std::vector<std::shared_ptr<Vertex> > margVertexs, int pose_dim) {

    SetOrdering();
    /// find the edges to marginalize, 
    // margVertexs[0] is frame, its edge contained pre-intergration
    std::vector<shared_ptr<Edge>> marg_edges = GetConnectedEdges(margVertexs[0]);
    
    // when find the marginalized landmark, find its 3D position in world and save it
    marginalized_pts.clear();
    current_pts.clear();

    std::unordered_map<int, shared_ptr<Vertex>> margLandmark;

    // build the Hessian matrix while keep the order of pose
    // the order of landmarks may change
    int marg_landmark_size = 0;
    // std::cout << "\n marg edge 1st id: "<< marg_edges.front()->Id() << " end id: "<<marg_edges.back()->Id()<<std::endl;
    for (size_t i = 0; i < marg_edges.size(); ++i) {
        // std::cout << "marg edge id: "<< marg_edges[i]->Id() <<std::endl;

        if(marg_edges[i]->TypeInfo() == string("EdgeReprojection")){
            // only the first verticies of edge reprojection is landmark vertex
            // and we are sure that this will contain all the verticies we want
            auto iter = marg_edges[i]->Verticies()[0];
            if(margLandmark.find(iter->Id()) == margLandmark.end()){
                iter->SetOrderingId(pose_dim + marg_landmark_size);
                margLandmark.insert(make_pair(iter->Id(), iter));
                //marg_landmark_size += iter->LocalDimension();
                // LocalDimension() is one for landmark verticies (inverse depth only)
                marg_landmark_size += 1;

                if(ifRecordMap){
                    // the vertex[1] of the edge i should be the host point from which we can extract its world position
                    // here we saved all the points connected to the frame
                    // however we should only save the points only owned by the frame
                    vector<shared_ptr<Edge>> remove_edges = GetConnectedEdges(iter);
                    Vec3 pts_w = (marg_edges[i])->GetPointInWorld(); 
                    if(remove_edges.size() == 1){
                        marginalized_pts.push_back(pts_w);
                    }
                    else{
                        current_pts.push_back(pts_w);
                    }
                }
            }
        }
/*
        // old version, loop all the verticies to find landmark verticies
        auto verticies = marg_edges[i]->Verticies();
        for (auto iter : verticies) {
            // if the vertex is a landmark and is not redundant in the margLandmark set
            if (IsLandmarkVertex(iter) && margLandmark.find(iter->Id()) == margLandmark.end()) {
                iter->SetOrderingId(pose_dim + marg_landmark_size);
                margLandmark.insert(make_pair(iter->Id(), iter));
                marg_landmark_size += iter->LocalDimension();
            }
        }
*/
    }

    //std::cout << "There are " << marginalized_pts.size() << " points to be marginalized. " << std::endl; 

    // the size of variables to be marginalized // pose_dim = 171
    int cols = pose_dim + marg_landmark_size;

    // std::cout << " marginalization variables size : " << cols << " pose_dim : " << pose_dim 
    //           << " marginalized pts : " << marginalized_pts.size() << std::endl;

    /// build the hessian matrix:  H = H_marg + H_pp_prior

    // H_marg has variables : { external_parameters,  poses , landmark_view_by_marginalized_frame )
    MatXX H_marg(MatXX::Zero(cols, cols));
    VecX b_marg(VecX::Zero(cols));
    int ii = 0;
    for (auto edge: marg_edges) {
        edge->ComputeResidual();
        edge->ComputeJacobians();
        auto jacobians = edge->Jacobians();
        auto verticies = edge->Verticies();
        ii++;

        assert(jacobians.size() == verticies.size());
        for (size_t i = 0; i < verticies.size(); ++i) {
            auto v_i = verticies[i];
            auto jacobian_i = jacobians[i];
            ulong index_i = v_i->OrderingId();
            ulong dim_i = v_i->LocalDimension();

            double drho;
            MatXX robustInfo(edge->Information().rows(),edge->Information().cols());
            edge->RobustInfo(drho,robustInfo);

            for (size_t j = i; j < verticies.size(); ++j) {
                auto v_j = verticies[j];
                auto jacobian_j = jacobians[j];
                ulong index_j = v_j->OrderingId();
                ulong dim_j = v_j->LocalDimension();

                MatXX hessian = jacobian_i.transpose() * robustInfo * jacobian_j;

                assert(hessian.rows() == v_i->LocalDimension() && hessian.cols() == v_j->LocalDimension());
                // add all the hessians
                H_marg.block(index_i, index_j, dim_i, dim_j) += hessian;
                if (j != i) {
                    // hessian is symmetry
                    H_marg.block(index_j, index_i, dim_j, dim_i) += hessian.transpose();
                }
            }
            b_marg.segment(index_i, dim_i) -= drho * jacobian_i.transpose() * edge->Information() * edge->Residual();
        }

    }
    if(verbose){
        std::cout << "edge factor cnt: " << ii <<std::endl;
    }

    /// marg landmark
    int reserve_size = pose_dim;
    if (marg_landmark_size > 0) {
        int marg_size = marg_landmark_size;
        MatXX Hmm = H_marg.block(reserve_size, reserve_size, marg_size, marg_size);
        MatXX Hpm = H_marg.block(0, reserve_size, reserve_size, marg_size);
        MatXX Hmp = H_marg.block(reserve_size, 0, marg_size, reserve_size);
        VecX bpp = b_marg.segment(0, reserve_size);
        VecX bmm = b_marg.segment(reserve_size, marg_size);

        // Hmm is digonal matrix, its inverse can be calculated by its blocks' inverses.
        MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
        // TODO:: use openMP to speed up
        for (auto iter: margLandmark) {
            int idx = iter.second->OrderingId() - reserve_size;
            int size = iter.second->LocalDimension();
            Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
        }

        MatXX tempH = Hpm * Hmm_inv;
        MatXX Hpp = H_marg.block(0, 0, reserve_size, reserve_size) - tempH * Hmp;
        bpp = bpp - tempH * bmm;
        H_marg = Hpp;
        b_marg = bpp;
    }

    VecX b_prior_before = b_prior_;
    if(H_prior_.rows() > 0)
    {
        H_marg += H_prior_;
        b_marg += b_prior_;
    }

    /// marg frame and speedbias
    int marg_dim = 0;

    // move the index from the tail of the vector
    for (int k = margVertexs.size() -1 ; k >= 0; --k)
    {

        int idx = margVertexs[k]->OrderingId();
        int dim = margVertexs[k]->LocalDimension();
        marg_dim += dim;
        // move the marginalized frame pose to the Hmm bottown right
        // 1. move row i the lowest part
        Eigen::MatrixXd temp_rows = H_marg.block(idx, 0, dim, reserve_size);
        Eigen::MatrixXd temp_botRows = H_marg.block(idx + dim, 0, reserve_size - idx - dim, reserve_size);
        H_marg.block(idx, 0, reserve_size - idx - dim, reserve_size) = temp_botRows;
        H_marg.block(reserve_size - dim, 0, dim, reserve_size) = temp_rows;

        // put col i to the rightest part
        Eigen::MatrixXd temp_cols = H_marg.block(0, idx, reserve_size, dim);
        Eigen::MatrixXd temp_rightCols = H_marg.block(0, idx + dim, reserve_size, reserve_size - idx - dim);
        H_marg.block(0, idx, reserve_size, reserve_size - idx - dim) = temp_rightCols;
        H_marg.block(0, reserve_size - dim, reserve_size, dim) = temp_cols;

        Eigen::VectorXd temp_b = b_marg.segment(idx, dim);
        Eigen::VectorXd temp_btail = b_marg.segment(idx + dim, reserve_size - idx - dim);
        b_marg.segment(idx, reserve_size - idx - dim) = temp_btail;
        b_marg.segment(reserve_size - dim, dim) = temp_b;
    }

    double eps = 1e-8;
    int m2 = marg_dim;
    int n2 = reserve_size - marg_dim;   // marg pose
    Eigen::MatrixXd Amm = 0.5 * (H_marg.block(n2, n2, m2, m2) + H_marg.block(n2, n2, m2, m2).transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd(
            (saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() *
                              saes.eigenvectors().transpose();

    Eigen::VectorXd bmm2 = b_marg.segment(n2, m2);
    Eigen::MatrixXd Arm = H_marg.block(0, n2, n2, m2);
    Eigen::MatrixXd Amr = H_marg.block(n2, 0, m2, n2);
    Eigen::MatrixXd Arr = H_marg.block(0, 0, n2, n2);
    Eigen::VectorXd brr = b_marg.segment(0, n2);
    Eigen::MatrixXd tempB = Arm * Amm_inv;
    // the rest pose become the new prior matrix 
    // upper we move the rest pose to the upper left, and new pose will be added at lower right. 
    // as a result the order of variables of the prior matrix is corresponding to the up coming Hessian.
    H_prior_ = Arr - tempB * Amr;
    b_prior_ = brr - tempB * bmm2;



    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(H_prior_);
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv = Eigen::VectorXd(
            (saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
    Jt_prior_inv_ = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    err_prior_ = -Jt_prior_inv_ * b_prior_;

    MatXX J = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    H_prior_ = J.transpose() * J;
    MatXX tmp_h = MatXX( (H_prior_.array().abs() > 1e-9).select(H_prior_.array(),0) );
    H_prior_ = tmp_h;

    // std::cout << "my marg b prior: " <<b_prior_.rows()<<" norm: "<< b_prior_.norm() << std::endl;
    // std::cout << "    error prior: " <<err_prior_.norm() << std::endl;

    // remove vertex and remove edge
    for (size_t k = 0; k < margVertexs.size(); ++k) {
        RemoveVertex(margVertexs[k]);
    }

    for (auto landmarkVertex: margLandmark) {
        RemoveVertex(landmarkVertex.second);
    }

    return true;

}


} // namespace backend
} // namespace myslam






