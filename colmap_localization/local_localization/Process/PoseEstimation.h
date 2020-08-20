#ifndef POSE_ESTIMATION_LOCAL_H_
#define POSE_ESTIMATION_LOCAL_H_

#include <iostream>
#include "SIFTExtractor.h"
#include "FeatureSift.h"
#include "MapTypes.h"
#include "SIFTMatcher.h"

#include <opencv2/opencv.hpp>

#include "Factor.h"

namespace BASTIAN
{

bool EstimatePosePnPRansac(CurrentFrame* pCurrentFrame, Eigen::Vector4d &qvec, Eigen::Vector3d &tvec);

bool OptimizeCameraPose(std::vector<cv::Point3f> &obj_pts, std::vector<cv::Point2f> &img_pts,
                          CurrentFrame* pCurrentFrame, Eigen::Vector4d &qvec, Eigen::Vector3d &tvec);

bool OptimizeCameraPose(std::vector<cv::Point3f> &obj_pts, std::vector<cv::Point2f> &img_pts, 
                          Eigen::Matrix4d &Twc_curr, PinholeCamera* pPinholeCamera);

} //namespace

#endif 
