#ifndef TOOLS_TEST_UTILS_H
#define TOOLS_TEST_UTILS_H

#include <fstream>

#include <pangolin/pangolin.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "util/logging.h"
#include "util/option_manager.h"

#include "base/reconstruction.h"
#include "base/database.h"
#include "base/projection.h"
#include "base/triangulation.h"

#include "estimators/pose.h"

#include "util/bitmap.h"
#include "util/misc.h"
#include "util/bitmap.h"
#include "util/ply.h"

#include "feature/sift.h"
#include "feature/matching.h"
#include "feature/utils.h"

#include "visual_index/visual_index_liuye.h"

// include opencv libraries, only used here in the example
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "configuration.h"

using namespace colmap;

namespace Ulocal{

// [TEST FCN] read the idx image's feature from database
FeatureKeypoints ReadDataBaseTest(const std::string& path, int idx = 276);

// SIFT extraction using CPU/GPU
void SIFTextractionTest(const std::string& pathimg,FeatureKeypoints* keypoints,FeatureDescriptors* descriptors, int new_width, int new_height);
bool SIFTextractionTestGPU(const std::string& pathimg,FeatureKeypoints* keypoints,FeatureDescriptors* descriptors, int new_width, int new_height);

// transform eigen matrix to opengl matrix for viewer
void GetOpenGLCameraMatrix(Eigen::Matrix3x4d matrix, pangolin::OpenGlMatrix &M);


class Ulocalization
{
public:
    Ulocalization();
    Ulocalization(const std::string& database_path, const std::string& recs_path);
    ~Ulocalization();

    // print the data info
    void PrintCameraInfo();

    // match the features with reference image(idx) using CPU or GPU
    FeatureMatches MatchWithImage(image_t idx, FeatureDescriptors &descriptors);
    void CreateGPUMatch();
    FeatureMatches MatchWithImageGPU(image_t idx, FeatureDescriptors &descriptors);

    // estimate pose: PnP + optimization
    bool PoseEstimation(FeatureKeypoints &keypoints,  image_t refIdx, FeatureMatches matches, 
                double focus_length, int width, int height);
    cv::Mat PoseEstimationCvMat(FeatureKeypoints &keypoints, image_t refIdx, 
                FeatureMatches matches, double focus_length, int width, int height);

    // using Vocabulary tree match algorithm (original Colmap version)
    void LoadVocTree(const std::string& index_path);
    void MakeVocTreeIndex(const std::string& tree_path,const std::string& write_path);
    void MakeVocTreeIndexForFloor(const std::string& tree_path,const std::string& write_path, int floor);

    image_t MatchVocTree(FeatureKeypoints &keypoints, FeatureDescriptors &descriptors);
    Retrieval MatchVocTreeReturnAll(FeatureKeypoints &keypoints, FeatureDescriptors &descriptors);

    // start viewer
    void View();

    // load a ply model file
    void LoadPLY(const std::string& ply_path);
    // load colmap MVS generated semi-dense point cloud
    void LoadFusedPointcloud(const std::string& load_path);
    void LoadPlyAndSaveJson(const std::string& load_path, const std::string& save_path);

    // set image file set
    void setImageFilePath(const std::string& set_path, const std::string& image_path);

    void SavePLYCLoud(const std::string& save_path);


private:
    Database* database;
    // all the image key frames
    std::vector<image_t> framesIds;

    std::string currentImagePath;
    std::string imageFilePath;

    Reconstruction* reconstruction;

    //sift matcher GPU
    bool siftMatcherGPUcreated = false;
    SiftMatchingOptions match_options;
    SiftMatchGPU* sift_match_gpu;

    // Voc tree
    VisualIndex_LY<> visual_index;

    int numCamera;
    Camera camera;

private:
    // fcns for pangolin viewer

    void testpangolin();
    
    void DrawCurrentframe();

    void DrawKeyframes();

    void DrawCoodinateSystem();

    void DrawPlane();

    void DrawTable(float ratio = 0.1);

    void DrawFused();

    void SetCurrentPose(Eigen::Vector4d qvec, Eigen::Vector3d tvec);

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


}  //namespace Ulocal

#endif //TOOLS_TEST_UTILS_H
