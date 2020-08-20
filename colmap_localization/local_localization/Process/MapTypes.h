#ifndef MAP_TYPES_H_LY_
#define MAP_TYPES_H_LY_

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "FeatureSift.h"

#define FRAME_GRID_ROWS 24
#define FRAME_GRID_COLS 32

namespace BASTIAN
{

struct PointMatches
{
    bool flag = false;
    Eigen::Vector3d mWorld;
    int distance = 999;
};

class LKeyFrame
{
public:
    LKeyFrame();
    ~LKeyFrame(){}

    void SetPose(Matrix3x4d mTwc_);

public:
    // world pose  -> InverseProjectionMatrix()
    Matrix3x4d mTwc;
    Eigen::Quaterniond qvec_cw;
    Eigen::Vector3d tvec_cw;

    // features
    int N;
    std::vector<FeatureKeypoint> vFeatureKeypoints;
    std::vector<FeatureDescriptor> vFeatureDescriptors;
    std::vector<Eigen::Vector3d> vPoseWorld;



    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();
    bool PosInGrid(const FeatureKeypoint &kp, int &posX, int &posY);

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    float mnMaxY = 0, mnMaxX = 0;
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    int SharedGridWithPoints(Eigen::Quaterniond &q_cw, Eigen::Vector3d &t_cw, Eigen::Matrix3d &mCalibration);
};

class CurrentFrame
{
public:
    CurrentFrame(){}
    CurrentFrame(std::vector<FeatureKeypoint> &vKeypoints_, FeatureDescriptors &mDescriptors_);
    ~CurrentFrame(){}
  
    void SetKeypoints(std::vector<FeatureKeypoint> &vKeypoints_, FeatureDescriptors &mDescriptors_);

    void SetCameraModel(float focus, int cols_, int rows_);

    bool InRange(Eigen::Vector2d pt);

    int last_matched_n_inliers = 0;
    void ClearMatches();
    int CountMatches();

    float radius = 15;
    bool FindBestMatch(Eigen::Vector2d &pt, FeatureDescriptor &mFeatureDescriptor, 
                       FeatureKeypoint &keypoint, Eigen::Vector3d &pt_world);

public:
    int cols, rows;
    Eigen::Matrix3d mCalibration;

    Eigen::Quaterniond qvec_cw;
    Eigen::Vector3d tvec_cw;

    int N;
    std::vector<FeatureKeypoint> vFeatureKeypoints;
    FeatureDescriptors mFeatureDescriptors;
    std::vector<PointMatches> vPointMatches;

    int nMatches = 0;
};


}




#endif
