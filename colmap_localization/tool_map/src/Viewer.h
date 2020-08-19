#ifndef VIEWER_LY_H_
#define VIEWER_LY_H_

#include "configuration.h"

namespace Ulocal{

class ViewerLY
{
public:
    ViewerLY(){}
    ~ViewerLY(){}

    void View();

    void SetMapPoints(std::vector<colmap::Point3D> &mapPoints_){
        mapPoints = mapPoints_;
    }

    void AddKeyFrame(Eigen::Matrix3x4d pose){
        pangolin::OpenGlMatrix Twc;
        GetOpenGLCameraMatrix(pose, Twc);
        Twcs.push_back(Twc);
    }

    void SetCurrentPose(Eigen::Vector4d qvec, Eigen::Vector3d tvec);

private:
    // fcns for pangolin viewer 

    void testpangolin();
   
    void DrawCurrentframe();

    void DrawKeyframes();

    void DrawCoodinateSystem();

    void DrawPlane();

    void DrawTable(float ratio = 0.1);

    void DrawFused();

private:

    void GetOpenGLCameraMatrix(Eigen::Matrix3x4d matrix, pangolin::OpenGlMatrix &M);

private:
    // info to draw in viewer

    // map 3d points
    std::vector<colmap::Point3D> mapPoints;
    std::vector<PlyPoint> plypoints;

    // keyframe opengl poses
    std::vector<pangolin::OpenGlMatrix> Twcs;
    std::vector<pangolin::OpenGlMatrix> currentTwcs;

    // variables for viewer

    float mViewpointX = 0.0F;
    float mViewpointY = -0.7F;
    float mViewpointZ = -1.8F;
    float mViewpointF = 500.0F;
    float mPointSize = 10.0F;
    float mKeyFrameSize = 0.05F;
    float mCameraLineWidth = 1.0F;
    float coordinateLength = 3.0F;
  
    int ndivs = 10;
    float ndivsize = 0.3F;
    float height = 1.0F;

    // for loaded mesh
    int numVertex, numFace;
    double *vertex_arrayX;
    double *vertex_arrayY;
    double *vertex_arrayZ;
    std::vector<int *> indexs;
    std::vector<float *> normals;

    float objectBottom = 0.0F;
    float objectX = -0.7F;
    float objectZ = 1.1F;
    float rotateY = 0.0F;

};

} // namespace

#endif


