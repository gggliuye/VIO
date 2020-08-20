#ifndef LY_SIFT_EXTRACTOR_H_
#define LY_SIFT_EXTRACTOR_H_

#include <iostream>
#include <array>
#include <fstream>
#include <memory>
#include <math.h>
#include <cfloat>

#include "configure.h"

extern "C"{
#include "VLFeat/generic.h"
#include "VLFeat/sift.h"
}

#include "FeatureSift.h"

#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
#include <android/log.h>

#define  LOG_TAG    "native-dev"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

*/
namespace BASTIAN
{

    struct SiftOptions{
        int max_num_features = 800;
        int num_octaves = 4;
        int octave_resolution = 3;
        int first_octave = -1;
        int max_num_orientations = 2;
        double peak_threshold = 0.00667;
        double edge_threshold = 10.0;

        // Fix the orientation to 0 for upright features.
        bool upright = false;

        enum class Normalization {
            // L1-normalizes each descriptor followed by element-wise square rooting.
            // This normalization is usually better than standard L2-normalization.
            // See "Three things everyone should know to improve object retrieval",
            // Relja Arandjelovic and Andrew Zisserman, CVPR 2012.
                    L1_ROOT,
            // Each vector is L2-normalized.
                    L2,
        };
        Normalization normalization = Normalization::L1_ROOT;
    };

    // need input gray image
    std::vector<float> ConvertToRowMajorArrayFloat(cv::Mat gray);

    bool SIFTExtractor(cv::Mat &imageGray, SiftOptions &options, bool bDes,
                       std::vector<FeatureKeypoint> &vOutputKeypoints, FeatureDescriptors &vOutputDescriptors);






} // namespace



#endif
