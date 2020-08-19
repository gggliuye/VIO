/*
*  @ brief : preprocessing the RGB images to reduce image variabilites
*  @ now : 
*          
*
*  @ author : Liu Ye
*  @ company : UTOPA, GuangZhou, China
*/

#ifndef IMAGEPERPROCESSING_H
#define IMAGEPERPROCESSING_H

#include <opencv2/opencv.hpp>

using namespace std;

class ImagePerprocessing
{

public:
  ImagePerprocessing(const string &strSettingsFile);

  void processingImage(cv::Mat &image);

  void gammaCorrection(cv::Mat &image);


private:
  void loadCfgFile(const string &strSettingsFile);
  cv::Mat * image;

  bool doGammacorrection = false;
  float fGamma = 1.0f;


};


#endif // IMAGEPERPROCESSING_H
