#ifndef ORB_MATCHER_UTOPA_H_
#define ORB_MATCHER_UTOPA_H_

#include "OrbFrame.h"
#include "SIFTMatcher.h"

namespace BASTIAN
{

void RefineMatchByDescriptors(OrbFrame* pOrbFrame_1, OrbFrame* pOrbFrame_2, 
                     std::vector<cv::Point2f> &pts_1,  std::vector<cv::Point2f> &pts_2, std::vector<int> &ids_1);



void TestMatchFramesFLANN(OrbFrame* pOrbFrame_1, OrbFrame* pOrbFrame_2, cv::Mat &image_show, bool bDraw = false);

void TestMatchFramesOpticalFlow(OrbFrame* pOrbFrame_1, OrbFrame* pOrbFrame_2, cv::Mat &image_show, bool bDraw = false);


void TryTriangulation(OrbFrame* pOrbFrame_1, OrbFrame* pOrbFrame_2, std::vector<cv::Point2f> &pts_2, std::vector<int> &ids_1);

int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

}



#endif
