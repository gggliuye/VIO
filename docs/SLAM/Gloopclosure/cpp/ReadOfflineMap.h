#ifndef READ_OFFLINE_MAP
#define READ_OFFLINE_MAP

#include <fstream>
#include <iostream>
#include <vector>
#include <map>

// opencv include
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

// LoopClosure include
#include "LoopClosure/KeyFrameMapPoint.h"
#include "LoopClosure/view.h"
#include "LoopClosure/orbextractor.h"
#include "LoopClosure/loopclosureparameters.h"
#include "LoopClosure/GMatcher.h"
#include "LoopClosure/PnPsolver.h"
#include "LoopClosure/GOptimization.h"

class OfflineMap
{

public:

    OfflineMap();
    ~OfflineMap(){}

    // read data from file
    void ReadMapFromDataFile(std::string &datafile);
    void ReadKeyFrame(std::ifstream &inputFile, GKeyFrame *pKF);
    void ReadMapPoint(std::ifstream &inputFile, GMapPoint *pKF);
    void ReIndexMapPoints();

    // DBOW2
    void LoadVocORB(const std::string &strVocFile);

    // localize new image
    void InitCameraParameters(float fx, float fy, float cx, float cy);
    bool LoopDetector(cv::Mat &image);    

    std::vector<int> matchKFs;

    void Draw();
    std::vector<cv::Mat> successedImages;

private:
    
    static const int TH_HIGH;
    static const int TH_MIN_MATCH_COUNT;

    //bool CheckGeometry(GKeyFrame &newImage, const fbow::fBow &newimagebow, const int oldIndex);

    std::vector<GKeyFrame*> DetectRelocalizationCandidates(GKeyFrame *F);

    std::vector<FBOWMatch> FindMatchDMatch(GKeyFrame &newImagekf, const int oldIndex);

    int SearchByProjection(GKeyFrame &F, const vector<GMapPoint*> &vpMapPoints, const float th);

    int SearchByProjection(GKeyFrame &CurrentFrame, GKeyFrame *pKF, const set<GMapPoint*> &sAlreadyFound, const float th , const int ORBdist);

public:
    // currentFrame
    // GKeyFrame newImage;

private:
    int keyframeId = 0;
    int currentId = 0;

    // DBOW2
    ORBVocabulary* mpVocabulary;

    // camera parameter for the new image
    cv::Mat mIntrinsic; 
    cv::Mat mDistortionCoeff; 
    float fx, fy, cx, cy;

    // feature extractor
    ORBextractor* pORBextractor;

    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Map info
    int keyframeSize;
    int countMapPoint;

    std::vector<GKeyFrame*> vKeyFrames;
    std::vector<GMapPoint*> vMapPoints;
    std::map<int, GMapPoint*> mapindices;

    // Inverted file
    std::vector<list<GKeyFrame*> > mvInvertedFile;

    // DMatcher
    cv::DescriptorMatcher *mMatcher;

    std::vector<cv::Point3f> point3D;
    std::vector<cv::Point2f> point2D;

    std::mutex mMutex;

};


#endif // #define READ_OFFLINE_MAP
