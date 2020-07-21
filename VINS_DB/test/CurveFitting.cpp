#include <iostream>
#include <random>
#include "backend/problem.h"
#include "utility/tic_toc.h"

using namespace myslam::backend;
using namespace std;

class CurveFittingVertex: public Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingVertex(): Vertex(3) {}  // abc:  Vertex dim 3
    virtual std::string TypeInfo() const { return "abc"; }
};

class CurveFittingEdge: public Edge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x, double y ): Edge(1,1, std::vector<std::string>{"abc"}) {
        x_ = x;
        y_ = y;
    }

    virtual void ComputeResidual() override
    {
        Vec3 abc = verticies_[0]->Parameters();  
        residual_(0) = std::exp( abc(0)*x_*x_ + abc(1)*x_ + abc(2) ) - y_;  
    }


    virtual void ComputeJacobians() override
    {
        Vec3 abc = verticies_[0]->Parameters();
        double exp_y = std::exp( abc(0)*x_*x_ + abc(1)*x_ + abc(2) );

        Eigen::Matrix<double, 1, 3> jaco_abc;  
        jaco_abc << x_ * x_ * exp_y, x_ * exp_y , 1 * exp_y;
        jacobians_[0] = jaco_abc;
    }
    virtual std::string TypeInfo() const override { return "CurveFittingEdge"; }
public:
    double x_,y_;  
};

void openmpTest1(){
    std::cout << std::endl << " -- OPENMP test1 -- " << std::endl;
    int data, id, total;
    // each thread has its own copy of data, id and total.
    #pragma omp parallel private(data, id, total) num_threads(6)
    {
      id = omp_get_thread_num();
      data = id; // threads may interleaving the modification
      total = omp_get_num_threads();
      printf("Greetings from process %d out of %d with Data %d\n", id, total, data);
    }
}

void openmpTest2(){
    std::cout << std::endl << " -- OPENMP test1 -- " << std::endl;
    double w_sigma= 1.;                
    float c[1000000];

    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.,w_sigma);

    TicToc t_test;
    for (int i = 0; i < 1000000; ++i){
        c[i] = noise(generator)*noise(generator)+noise(generator)*noise(generator);
    }
    std::cout << t_test.toc() << std::endl;

    TicToc t_test2;
    #pragma omp parallel for
    for (int i = 0; i < 1000000; ++i){
        c[i] = noise(generator)*noise(generator)+noise(generator)*noise(generator);
    }
    std::cout << t_test2.toc() << std::endl;
}


int main()
{
    double a=1.0, b=2.0, c=1.0;         
    int N = 100;                        
    double w_sigma= 1.;                

    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.,w_sigma);

    Problem problem(Problem::ProblemType::GENERIC_PROBLEM);
    shared_ptr< CurveFittingVertex > vertex(new CurveFittingVertex());

    vertex->SetParameters(Eigen::Vector3d (0.,0.,0.));
    problem.AddVertex(vertex);

    for (int i = 0; i < N; ++i) {

        double x = i/100.;
        double n = noise(generator);
        double y = std::exp( a*x*x + b*x + c ) + n;
//        double y = std::exp( a*x*x + b*x + c );

        shared_ptr< CurveFittingEdge > edge(new CurveFittingEdge(x,y));
        std::vector<std::shared_ptr<Vertex>> edge_vertex;
        edge_vertex.push_back(vertex);
        edge->SetVertex(edge_vertex);

        problem.AddEdge(edge);
    }

    std::cout<<"\nTest CurveFitting start..."<<std::endl;
    problem.Solve(30);

    std::cout << "-------After optimization, we got these parameters :" << std::endl;
    std::cout << vertex->Parameters().transpose() << std::endl;
    std::cout << "-------ground truth: " << std::endl;
    std::cout << "1.0,  2.0,  1.0" << std::endl;


    openmpTest1();

    openmpTest2();

    return 0;
}


