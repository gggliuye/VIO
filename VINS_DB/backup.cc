//////////////////////////////////////////////////
//////////////////////////////////////////////////
// remake the elements into a vector 
// then use openmp
/////////////////////////////////////////////////


void Problem::MakeHessian() {
    TicToc t_h;
    // build the large Hessian matrix
    ulong size = ordering_generic_;
    MatXX H(MatXX::Zero(size, size));
    VecX b(VecX::Zero(size));

    std::vector<std::shared_ptr<Edge>> vEdges;

    for (auto edge: edges_) {
        vEdges.push_back(edge.second);
    }

    int eSize = vEdges.size();
    //std::cout << " size vector : " << eSize << std::endl;
    int k = 0;
    #pragma omp parallel for
    for (k = 0; k < eSize; k ++) {

        vEdges[k]->ComputeResidual();
        vEdges[k]->ComputeJacobians();

        // TODO:: robust cost
        auto jacobians = vEdges[k]->Jacobians();
        auto verticies = vEdges[k]->Verticies();
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
            MatXX robustInfo(vEdges[k]->Information().rows(),vEdges[k]->Information().cols());
            vEdges[k]->RobustInfo(drho,robustInfo);

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
            b.segment(index_i, dim_i).noalias() -= drho * jacobian_i.transpose()* vEdges[k]->Information() * vEdges[k]->Residual();
        }
    }

    vEdges.clear();

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



/////////////////////////////////////
/////////////////////////////////////
//////////  ORIGINAL  ///////////////
/////////////////////////////////////

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

    // TODO:: accelate, accelate, accelate
//#ifdef USE_OPENMP
//#pragma omp parallel for
//#endif

    //#pragma omp parallel 
    for (auto &edge: edges_) {

        edge.second->ComputeResidual();
        edge.second->ComputeJacobians();

        // TODO:: robust cost
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
                    H.block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();
                }
            }
            #pragma omp critical
            b.segment(index_i, dim_i).noalias() -= drho * jacobian_i.transpose()* edge.second->Information() * edge.second->Residual();
        }
    }

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
