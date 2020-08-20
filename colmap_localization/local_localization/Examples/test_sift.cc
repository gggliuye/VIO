#include <iostream>
#include <string>

#include "SIFTExtractor.h"
#include <opencv2/highgui/highgui.hpp>

using namespace BASTIAN;
using namespace std;

int testMain(int video_id);

int main(int argc, char **argv)
{
    if(argc != 2){
        std::cout << "./test_superpoint video_id \n";
        std::cout << "cpulimit -l 100 ./test_superpoint video_id \n";
        return 0;
    }

    int video_id = atoi(argv[1]);

    testMain(video_id);
    return 0;
}

int testMain(int video_id)
{
    cv::VideoCapture cap(video_id);
    if(!cap.isOpened()){
      cout << "Fail to get camera" << endl;
      return -1;
    }

    SiftOptions options;
    //std::cout << options.max_num_features << std::endl;


    // Main loop
    cv::Mat im;
    while(true){
        cap >> im;
        if(im.empty()){
            cerr << endl << "Failed to read image. " << endl;
            break;
        }
        cv::Mat imageGray;
        cvtColor(im, imageGray, CV_RGB2GRAY);

        std::vector<FeatureKeypoint> vOutputKeypoints;
        FeatureDescriptors vOutputDescriptors;
        SIFTExtractor(imageGray, options, true, vOutputKeypoints, vOutputDescriptors);

        for(size_t i = 0 ; i < vOutputKeypoints.size(); i ++){
            cv::circle(im, cv::Point2f(vOutputKeypoints[i].x, vOutputKeypoints[i].y), 5, cv::Scalar(255,0,0),0);
        }

        //std::cout << " Number of features : " << vOutputKeypoints.size() << std::endl;

        cv::imshow("vlf sift point test", im);
        cv::waitKey(10);
    }

    return 0;
}
