#ifndef LOCAL_LOCALIZATION_H_
#define LOCAL_LOCALIZATION_H_

#include <fstream>
#include <iostream>

#include "SIFTExtractor.h"
#include "FeatureSift.h"
#include "MapTypes.h"
#include "SIFTMatcher.h"
#include "PoseEstimation.h"

namespace BASTIAN
{

struct SearchResult {
    SearchResult(int id_, int count_){
        id = id_; count = count_;
    }

    int id;
    int count;
};

class LocalLocalization
{

public:
    LocalLocalization(const std::string &map_path);
    ~LocalLocalization();

    bool LocalizeImage(cv::Mat &imageGray, float focus, Eigen::Vector4d &qvec, Eigen::Vector3d& tvec);

    bool LocalizeImageFLANN(cv::Mat &imageGray, float focus, Eigen::Vector4d &qvec, Eigen::Vector3d& tvec);

    // find candidate keyframes
    std::vector<int> FindCandidateKeyFrames(Eigen::Quaterniond qvec, Eigen::Vector3d tvec);
    std::vector<SearchResult> FindCandidateKeyFramesV2(Eigen::Quaterniond qvec, Eigen::Vector3d tvec, Eigen::Matrix3d &mCalibration);

private:
    double Threshold_angle = 15;
    double Threshold_distance = 1.5;
    double CompareQuaternions(Eigen::Quaterniond q1, Eigen::Quaterniond q2);

public:
    // load map from file
    LKeyFrame* LoadKeyFrame(std::ifstream &inputFile);
    bool ReadMap(const std::string &load_path);

    void SaveMapPly(const std::string &save_path);

private:
    SiftOptions options;

    // map data
    std::vector<cv::Mat> vDescriptors;
    std::vector<LKeyFrame*> vLKeyFrames;

public:
    // only for show
    CurrentFrame* pLastFrame;
    
    // only for debug log
    bool b_debug = false;
    std::ofstream runtimefile;
    void SetFile(std::string file_name){
        b_debug = true;
        runtimefile.open(file_name.c_str());
        runtimefile << std::fixed;
    }

};






}




#endif
