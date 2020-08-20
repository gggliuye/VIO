/*
* Designed for Phone usage.
* We should have a petty good estimation of the current pose (eg. pose tracking using ARCore)
* 0. Find the closest keyframe in our database (both position and direction)
* 1. Use this pose, we can project the matched keyframe points into our current frame.
* 2. Find matches of SIFT points, within a small range.
* 3. Optimize the pose using least square.
*
* For this, we need:
* 1. Input the current pose estimation in the map frame.
* 2. A map database, with 3d points, and frame poses.
*
* We can also check if the frame is too far from all the keyframes, we can ignore it.
* 
*/


#ifndef LY_SIFT_MATHCER_H_
#define LY_SIFT_MATHCER_H_

#include <iostream>
#include "SIFTExtractor.h"
#include "FeatureSift.h"
#include "MapTypes.h"

#include <opencv2/opencv.hpp>

namespace BASTIAN
{


// match the points in a close region 
bool MatchFrames(CurrentFrame* pCurrentFrame, LKeyFrame* pKeyFrame, double &radius);

// match features using FLANN
std::vector<cv::DMatch> FlannOneWay(cv::Mat &descriptor_curr, cv::Mat &descriptor_ref);

std::vector<cv::DMatch> MatchFramesFLANN(CurrentFrame* pCurrentFrame, LKeyFrame* pKeyFrame);

std::vector<cv::DMatch> MatchFramesFLANN(cv::Mat &descriptor_curr, cv::Mat &descriptor_ref);

}


#endif
