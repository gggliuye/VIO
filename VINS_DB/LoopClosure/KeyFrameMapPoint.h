#ifndef KEYFRAME_MAPPOINT
#define KEYFRAME_MAPPOINT

#include <opencv2/core/core.hpp>
#include <vector>
#include <mutex>

// opencv include
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

// loop closure project
#include "LoopClosure/DBoW2/DBoW2/FORB.h"
#include "LoopClosure/DBoW2/DBoW2/TemplatedVocabulary.h"
#include "LoopClosure/DBoW2/DBoW2/BowVector.h"
#include "LoopClosure/DBoW2/DBoW2/FeatureVector.h"

#include "LoopClosure/Converter.h"
#include "LoopClosure/loopclosureparameters.h"

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
  ORBVocabulary;


class GMapPoint
{

public:

    GMapPoint(){}
    ~GMapPoint(){}


    int id;
    int nObs;

    cv::Mat worldPose;
    cv::Point3f worldPoint;
    std::vector<int> observations;
    
private:
    // remember : mutex no not allow copy, as a result, we can only have one copy of it
    std::mutex mMutexPos;


public:
    cv::Mat GetWorldPos(){ 
        unique_lock<mutex> lock(mMutexPos);
        return worldPose.clone();
    }

};

class GKeyFrame
{
public :

    GKeyFrame(): mnLoopQuery(0), mnLoopWords(0), mLoopScore(0), mnRelocQuery(0), mnRelocWords(0), mRelocScore(0){}
    ~GKeyFrame(){}
    
    int id;
    cv::Mat Tcw;    // Tcw   
    cv::Mat Twc;   // Twc
    cv::Mat Ow;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<int> mpIds;
    // MapPoints associated to keypoints
    std::vector<GMapPoint*> mvpMapPoints;

public:
    // BoW
    //KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Variables used by the keyframe database
    int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    vector<float> mvLevelSigma2;
    std::mutex mMutexFeatures;

    // only for current frame
    double fx, fy, cx, cy;
    float mnMinX = 0;
    float mnMinY = 0;
    float mnMaxX = 480;
    float mnMaxY = 640;

public:
    void SetPose(const cv::Mat &Tcw_)
    {
        Tcw_.copyTo(Tcw);
        cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = Tcw.rowRange(0,3).col(3);
        cv::Mat Rwc = Rcw.t();
        Ow = -Rwc*tcw;

        Twc = cv::Mat::eye(4,4,Tcw.type());
        Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
        Ow.copyTo(Twc.rowRange(0,3).col(3));
    }

    void SetVocabulary(ORBVocabulary* mpORBvocabulary_)
    {
        mpORBvocabulary = mpORBvocabulary_;
    }

    void ComputeBoW()
    {
        if(mBowVec.empty() || mFeatVec.empty())
        {
            std::vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(descriptors);
            // Feature vector associate features with nodes in the 4th level (from leaves up)
            // We assume the vocabulary tree has 6 levels, change the 4 otherwise
            mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4); 
        }
    }

    vector<GMapPoint*> GetMapPointMatches()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints;
    }

    void SetCamera(double fx_, double fy_, double cx_, double cy_)
    {
        fx = fx_; fy = fy_; cx = cx_; cy = cy_;
    }

    void ResetmvpMapPoints()
    {
        mvpMapPoints.clear();
        mvpMapPoints.reserve(keypoints.size());
        for(size_t i = 0; i < keypoints.size() ; i++)
            mvpMapPoints.push_back(static_cast<GMapPoint*>(NULL));
    }

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int &nPredictedLevel)
    {
        vector<size_t> vIndices;
        int N = keypoints.size();
        vIndices.reserve(N);

        for(int i = 0; i < N ; i++){
            const cv::KeyPoint &kpUn = keypoints[i];
            const float distx = kpUn.pt.x-x;
            const float disty = kpUn.pt.y-y;
            //if(kpUn.pt.x < maxX && kpUn.pt.x > minX && kpUn.pt.y < maxY && kpUn.pt.y > minY)
            if(kpUn.octave > nPredictedLevel + 1 || kpUn.octave < nPredictedLevel - 1)
                continue;

            if(fabs(distx)<r && fabs(disty)<r)
                vIndices.push_back(i);
        }
        return vIndices;
    }


};  // GKeyFrame



struct FBOWMatch
{
    FBOWMatch(int a, int b, int c){index1 = a; index2 = b; distance = c;}

    int index1;
    int index2;
    int distance;
};

/*
struct BOWResult
{
    int index;
    double score;
};

// compare the score of the result
// define the operator for using std::sort
bool operator<(const BOWResult &a, const BOWResult &b) {
  return a.score > b.score;
}
*/

#endif // #define KEYFRAME_MAPPOINT
