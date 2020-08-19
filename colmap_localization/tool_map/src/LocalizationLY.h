#ifndef LOCALIZATION_LY_H_
#define LOCALIZATION_LY_H_

#include "Viewer.h"
#include "configuration.h"

#include "SiftGPU/SiftGPU.h"

#define SAVE_TIME 0

namespace Ulocal{

using namespace colmap;

class LocalizationLY
{

public:
    LocalizationLY(const std::string& database_path, const std::string& recs_path, const std::string& index_path,
                       bool bViewer_=false, bool bSaveResult_=false);
    ~LocalizationLY();

    void View();

    bool LocalizeImage(cv::Mat &image, double focus_length, Eigen::Vector4d &qvec, Eigen::Vector3d &tvec, bool bAsInitEstimate = false);

    void LoadVocTree(const std::string& index_path);

private:

    bool SIFTextractionGPU(cv::Mat &image, FeatureKeypoints* keypoints,
                FeatureDescriptors* descriptors);

    bool SIFTextractionCPU(cv::Mat &image, FeatureKeypoints* keypoints,
                FeatureDescriptors* descriptors);

    Retrieval MatchVocTreeReturnAll(FeatureKeypoints &keypoints, FeatureDescriptors &descriptors);

    double CompareQuaternions(Eigen::Vector4d &qvec_1, Eigen::Vector4d &qvec_2);
    Retrieval MatchWithInitialPoseEstimation(Eigen::Vector4d &qvec, Eigen::Vector3d &tvec);

    FeatureMatches MatchWithImageGPU(image_t idx, FeatureDescriptors &descriptors);

    FeatureMatches MatchWithImageCPU(image_t idx, FeatureDescriptors &descriptors);

    bool PoseEstimation(FeatureKeypoints &keypoints, image_t refIdx, FeatureMatches matches, 
                        double focus_length, int width, int height, Eigen::Vector4d &qvec, Eigen::Vector3d &tvec);

public:
    // image database
    int numImage;
    Database* database;

    // camera models
    int numCamera;

    // sparse reconstruction data
    Reconstruction* reconstruction;

    // Voc tree
    VisualIndex_LY<> visual_index;

    //sift matcher GPU
    bool siftMatcherGPUcreated = false;
    SiftMatchingOptions match_options;
    SiftMatchGPU* sift_match_gpu;

    // match with init pose estimation
    std::vector<image_t> framesIds;
    double Threshold_angle = 60;
    double Threshold_distance = 1;

    // candidate match
    int maxTrival = 10;
    int maxTrival_init_pose = 20;

public:
    bool bVerbose = true;

    bool bSaveResult;
    std::string runtimeFilename;
    std::ofstream runtimefile;


    bool bViewer;
    ViewerLY *pViewerLY;

};

}  //namespace Ulocal


#endif // LOCALIZATION_LY_H_
