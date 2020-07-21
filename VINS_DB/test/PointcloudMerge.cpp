#include <iostream>
#include <random>
#include "backend/problem.h"
#include "backend/vertex_scale.h"
#include "backend/vertex_pose.h"
#include "backend/edge_mapfusion.h"

#include <pangolin/pangolin.h>

#define PI 3.14159625

using namespace myslam::backend;
using namespace std;

vector<Vec3> pts_i_all;
vector<Vec3> pts_j_all;
vector<Vec3> pts_i_new;
vector<Vec3> pts_i_SVD;

double pose_estimation_SVD(
	const vector<Vec3>& pts1,
	const vector<Vec3>& pts2,
	Eigen::Matrix3d& R, Vec3& t);

void makeTestPointPair();

void testpangolin();

void poseOptimization(const vector<Vec3>& pts_i_all, const vector<Vec3>& pts_j_all,
	Eigen::Matrix3d& R_, Vec3& t_, double &scale_);

void solveMatrixTransformation();


int main()
{ 
    solveMatrixTransformation();

    std::cout << "Load the point pairs " << std::endl;     
    makeTestPointPair();
    int pointSize = pts_i_all.size();

    std::cout << "Estimate with SVD : " << std::endl;
    Eigen::Matrix3d R;
    Vec3 t;
    double scale_SVD = pose_estimation_SVD(pts_j_all, pts_i_all, R, t);

    Qd qq;
    qq = R; 

    std::cout << t.transpose() << " " << qq.x() << " " << qq.y() << " " << qq.z()
              << " " << qq.w() << std::endl;
    std::cout << " " << scale_SVD << std::endl;

    for(int i = 0; i < pointSize; i++){
        Vec3 point_i = pts_i_all[i];

        Vec3 point_i_new = scale_SVD * (R * point_i + t);
        pts_i_SVD.push_back(point_i_new);
    }

        
    poseOptimization(pts_i_all, pts_j_all, R, t, scale_SVD);

    for(int i = 0; i < pointSize; i++){
        Vec3 point_i = pts_i_all[i];

        Vec3 point_i_new = scale_SVD * (R * point_i + t);
        pts_i_new.push_back(point_i_new);
    }

    qq = R; 

    std::cout << t.transpose() << " " << qq.x() << " " << qq.y() << " " << qq.z()
              << " " << qq.w() << std::endl;
    std::cout << " " << scale_SVD << std::endl;
/*
    Qd test;
    test.x() = 0.707107;
    test.y() = -0.707107;
    test.z() = 0;
    test.w() = 0;

    std::cout << test.matrix() << std::endl << std::endl;
*/
    //testpangolin();
    
    return 0;
}


void solveMatrixTransformation()
{
    std::cout << " Make two sets of points ..." << std::endl;

    //the points in VINS frame
    std::vector<Vec3> pts_vins;
    {
    Vec3 pts1; pts1 << 0,0,0;
    pts_vins.push_back(pts1);
    Vec3 pts2; pts2 << 1,0,0;
    pts_vins.push_back(pts2);
    Vec3 pts3; pts3 << 0,-1,0;
    pts_vins.push_back(pts3);
    Vec3 pts4; pts4 << 0,0,1;
    pts_vins.push_back(pts4);
    }

    //the points in unity frame
    std::vector<Vec3> pts_unity;
    {
    Vec3 pts1; pts1 << 0,0,0;
    pts_unity.push_back(pts1);
    Vec3 pts2; pts2 << 0,0,1;
    pts_unity.push_back(pts2);
    Vec3 pts3; pts3 << -1,0,0;
    pts_unity.push_back(pts3);
    Vec3 pts4; pts4 << 0,1,0;
    pts_unity.push_back(pts4);
    }

    Eigen::Matrix3d R;
    Vec3 t;
    Qd qq;

    // test mirror transform
    R << -1,0,0,0,1,0,0,0,1;
    qq = R;
    
    std::cout << " mirror q : " << qq.x() << " " << qq.y() << " " << qq.z()
              << " " << qq.w() << std::endl << std::endl;

    std::cout << qq.matrix() << std::endl << std::endl;

    // SVD solve
    std::cout << " Estimate with SVD : " << std::endl;

    double scale_SVD = pose_estimation_SVD(pts_unity, pts_vins, R, t);

    qq = R; 

    std::cout << t.transpose() << " " << qq.x() << " " << qq.y() << " " << qq.z()
              << " " << qq.w() << std::endl;
    std::cout << qq.matrix() << std::endl;
    std::cout << " " << scale_SVD << std::endl << std::endl;


/*
    std::cout << " Estimate with optimization : " << std::endl;

    double tmp = 1;
    t << 0,0,0;
    poseOptimization(pts_vins, pts_unity, R, t, tmp);

    qq = R; 

    std::cout << " t : " << t.transpose() << " q : " << qq.x() << " " << qq.y() << " " << qq.z()
              << " " << qq.w() << std::endl << std::endl;
*/
}


void poseOptimization(const vector<Vec3>& pts_i_all, const vector<Vec3>& pts_j_all,
	Eigen::Matrix3d& R_, Vec3& t_, double &scale_)
{
    // initialize the problem
    Problem problem(Problem::ProblemType::GENERIC_PROBLEM);
    shared_ptr<VertexPose> vertexPose(new VertexPose());
    shared_ptr<VertexScale> vertexScale(new VertexScale());

    // set the initial values of vertex
    Qd qq;
    qq = R_; 
    Eigen::VectorXd pose(7);
    pose << t_(0), t_(1), t_(2), qq.x(), qq.y(), qq.z(), qq.w();
    vertexPose->SetParameters(pose);
    Eigen::VectorXd scale(1);
    scale << scale_;
    vertexScale->SetParameters(scale);

    // add the vertices
    problem.AddVertex(vertexPose);
    problem.AddVertex(vertexScale);

    int pointSize = pts_i_all.size();
   
    for(int i = 0; i < pointSize; i++){
        Vec3 pts_i = pts_i_all[i];
        Vec3 pts_j = pts_j_all[i];
        shared_ptr< EdgeMapFusion > edge(new EdgeMapFusion(pts_i,pts_j));

        std::vector<std::shared_ptr<Vertex>> edge_vertex;
        edge_vertex.push_back(vertexPose);
        edge_vertex.push_back(vertexScale);
        edge->SetVertex(edge_vertex);

        problem.AddEdge(edge);
    }


    std::cout<<"\n Nonlinear Optimzation Merge Point Cloud Start. "<<std::endl;

    problem.Solve(40);

/*
    std::cout << "-------After optimization, we got these parameters :" << std::endl;
    std::cout << vertexPose->Parameters().transpose() << std::endl;
    std::cout << vertexScale->Parameters().transpose() << std::endl << std::endl;
*/

    Eigen::VectorXd groundTruth = vertexPose->Parameters();
    Qd groundTruthQ(groundTruth[6], groundTruth[3], groundTruth[4], groundTruth[5]);
    groundTruthQ.normalize();
    //double scale_new = vertexScale->Parameters()[0];

    R_ = groundTruthQ.matrix();
    t_ << groundTruth[0], groundTruth[1], groundTruth[2];
    scale_ = vertexScale->Parameters()[0];
}


double pose_estimation_SVD(
	const vector<Vec3>& pts1,
	const vector<Vec3>& pts2,
	Eigen::Matrix3d& R, Vec3& t)
{
        // calculate p1 p2
	Vec3 p1, p2;
	int N = pts1.size();
	for (int i = 0; i<N; i++){
		p1 += pts1[i];
		p2 += pts2[i];
	}
	p1 = Vec3((p1) / N);
	p2 = Vec3((p2) / N);
    //std::cout << p1 << p2 ;

	vector<Vec3> q1(N), q2(N); 
	for (int i = 0; i<N; i++){
		q1[i] = pts1[i] - p1;
		q2[i] = pts2[i] - p2;
	}
 
	// 2. W = sum(q1*q2^T)
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for (int i = 0; i<N; i++){
		W += q1[i] * q2[i].transpose();
        //std::cout << W << std::endl;
	}

	// 3. SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
    //std::cout << U << std::endl;
    //std::cout << V << std::endl;
    
	// 4. R = U*V^T
	R = U * (V.transpose());

    // 5. scale
    double scale = 0;
    for(int i = 0 ; i < N ; i ++){
        Vec3 tmp = R * q2[i];
        for(int k = 0 ; k < 3 ; k++){
            scale += q1[i](k)/tmp(k);
        }
    }
    scale /= (3*N);

    // 6. t
	t = p1 / scale - R * p2;
 
    return scale;
}

void makeTestPointPair()
{
    pts_i_all.clear();
    pts_j_all.clear();

    int numberPair = 200;
    
    double simulateRadius = 10;

    double w_sigma= 0.2;
    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.,w_sigma);

    std::default_random_engine generatorP;
    std::normal_distribution<double> pointP(0.,simulateRadius);


    Eigen::VectorXd groundTruth(7);
    groundTruth << 20, -20, -1.2, 0.25774, 0.448701, 0.460485, 0.721243;
    double scale = 1.2;

    Qd groundTruthQ(groundTruth[6], groundTruth[3], groundTruth[4], groundTruth[5]);
    groundTruthQ.normalize();

    for(int i = 0; i < numberPair; i++){
        Vec3 point_i;
        //point_i << simulateRadius*cos(2*PI*i/numberPair), simulateRadius*sin(2*PI*i/numberPair), 10;
        point_i << pointP(generator), pointP(generator), pointP(generator);
        //double n = noise(generator);
        Vec3 noiseVec(noise(generator), noise(generator), noise(generator));
        Vec3 point_j = scale * (groundTruthQ * point_i + Vec3(groundTruth[0], groundTruth[1], groundTruth[2])) + noiseVec;

        pts_i_all.push_back(point_i);
        pts_j_all.push_back(point_j);
        //std::cout << point_i.transpose() << " <-> " << point_j.transpose() << std::endl;
    }

    std::cout << "Ground Truth is : " << groundTruth[0] << " " << groundTruth[1] << " " << groundTruth[2] 
              << " " << groundTruthQ.x() << " " << groundTruthQ.y() << " " << groundTruthQ.z()
              << " " << groundTruthQ.w() << std::endl
              << "                  scale : " << scale << std::endl;

}



void testpangolin()
{

    pangolin::CreateWindowAndBind("Colmap Ulocal: Map Viewer",1224,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(200));
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);

    float mViewpointX = 0.0F;
    float mViewpointY = -0.7F*10;
    float mViewpointZ = -1.8F*10;
    float mViewpointF = 500.0F;

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
 
    while(!pangolin::ShouldQuit())
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        d_cam.Activate(s_cam);

        if(menuShowPoints)
        {
            // draw map points
            glPointSize(5);
            glBegin(GL_POINTS);
            for(size_t i=0, iend=pts_i_all.size(); i<iend;i++)
            {
                glColor3f(0,0,0);
                glVertex3f(pts_i_all[i](0),pts_i_all[i](1),pts_i_all[i](2));
                glColor3f(1,1,0);
                glVertex3f(pts_j_all[i](0),pts_j_all[i](1),pts_j_all[i](2));
                glColor3f(0,0,1);
                glVertex3f(pts_i_new[i](0),pts_i_new[i](1),pts_i_new[i](2));
                glColor3f(1,0,0);
                glVertex3f(pts_i_SVD[i](0),pts_i_SVD[i](1),pts_i_SVD[i](2));
            }
            glEnd();
        }
 
        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
}
