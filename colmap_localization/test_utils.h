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
//#include "feature/feature.cc"
#include "feature/utils.h"

#include "retrieval/visual_index.h"

// FBOW include
#include "fbow.h"
#include "vocabulary_creator.h"

// include opencv libraries, only used here in the example
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace colmap;

namespace Ulocal{


FeatureKeypoints ReadDataBaseTest(const std::string& path, int idx = 276);

void SIFTextractionTest(const std::string& pathimg,FeatureKeypoints* keypoints,FeatureDescriptors* descriptors, int new_width, int new_height);

void GetOpenGLCameraMatrix(Eigen::Matrix3x4d matrix, pangolin::OpenGlMatrix &M);


class Ulocalization
{
public:
    Ulocalization();
    Ulocalization(const std::string& database_path, const std::string& recs_path);
    ~Ulocalization();

    void OpenDatabase(const std::string& path);

    void ReadDataBaseTest(image_t idx, FeatureKeypoints &keypoints,FeatureDescriptors &descriptors);

    FeatureMatches MatchWithImage(image_t idx, FeatureDescriptors &descriptors);

    bool PoseEstimation(FeatureKeypoints &keypoints,  image_t refIdx, FeatureMatches matches);

    void View();

    void MakeVocTreeIndex(const std::string& tree_path,const std::string& write_path);

    image_t MatchVocTree(const std::string& index_path, FeatureKeypoints &keypoints, FeatureDescriptors &descriptors);

    void LoadPLY(const std::string& ply_path);

    void MakeFBOWvocabulary(const std::string& save_path);
    
    void LoadFBOWvocabulary(const std::string& load_path);

    void AddImagesToVoc();

    void MatchImage(FeatureDescriptors &descriptors);

    void LoadFusedPointcloud(const std::string& load_path);

private:

    void testpangolin();
    
    void DrawCurrentframe();

    void DrawKeyframes();

    void DrawCoodinateSystem();

    void DrawPlane();

    void DrawTable(float ratio = 0.1);

    void DrawFused();

    void SetCurrentPose(Eigen::Vector4d qvec, Eigen::Vector3d tvec);


private:
    Database* database;

    Reconstruction* reconstruction;

    fbow::Vocabulary voc;
    std::vector<fbow::fBow> keyframesBOW;

    Camera camera;

    // map 3d points
    std::vector<colmap::Point3D> mapPoints;
    std::vector<PlyPoint> plypoints;

    std::vector<image_t> framesIds;
    // keyframe opengl poses
    std::vector<pangolin::OpenGlMatrix> Twcs;

    pangolin::OpenGlMatrix currentTwc;

private:

    // for viewer
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

struct Retrieval {
  image_t image_id = kInvalidImageId;
  std::vector<retrieval::ImageScore> image_scores;
};


}  //namespace Ulocal

#endif //TOOLS_TEST_UTILS_H
