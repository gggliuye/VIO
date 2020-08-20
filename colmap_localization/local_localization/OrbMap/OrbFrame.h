#ifndef ORB_FRAME_UTOPA_H_
#define ORB_FRAME_UTOPA_H_

#include <vector>

#include "configure.h"
#include "orbextractor.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include "Factor.h"
#include "OrbPoint.h"

#define FRAME_GRID_ROWS_ORB 48
#define FRAME_GRID_COLS_ORB 64

namespace BASTIAN
{

class OrbFrame
{

public:
    OrbFrame(std::string &image_file_, ORBextractor* pORBextractor, PinholeCamera* pPinholeCamera_);
    bool InBorder(const cv::Point2f &pt);

    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1);


public:
    // using simple pinhole camera model -> no distortion
    PinholeCamera* pPinholeCamera;

    std::string image_file;
    int N;
    cv::Mat mDescriptors;
    std::vector<cv::KeyPoint> vKeyPoints;
    std::vector<OrbPoint*> vpOrbPoints;
    Eigen::Matrix4d mPose;

private:
    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();
    bool PosInGrid(const cv::Point2f &kp, int &posX, int &posY);

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    float mnMaxY = 0, mnMaxX = 0;
    float mnMinX = 0, mnMinY = 0;
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS_ORB][FRAME_GRID_ROWS_ORB];

};

} // namespace BASTIAN


#endif
