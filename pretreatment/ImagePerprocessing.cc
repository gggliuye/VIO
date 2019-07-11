#include "ImagePerprocessing.h"
#include <iomanip>


ImagePerprocessing::ImagePerprocessing(const string &strSettingsFile)
{
    loadCfgFile(strSettingsFile);
}



void ImagePerprocessing::loadCfgFile(const string &strSettingsFile)
{
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << "  use default setttings. "<< endl;
    }

    // gamma correction of brightness
    fGamma = fsSettings["Brigthness.Gamma"];
    cv::FileNode ffdoGammacorrection = fsSettings["Brigthness.DoIt"];
    if((string)ffdoGammacorrection == "true")
    {  doGammacorrection = true;}
    else
    {  doGammacorrection = false;}
} 


void ImagePerprocessing::processingImage(cv::Mat &image)
{
    if(doGammacorrection)
    {
        gammaCorrection(image);
    }

}

void ImagePerprocessing::gammaCorrection(cv::Mat &image)
{
    unsigned char lut[256];
 
    for (int i = 0; i < 256; i++)
    {
        lut[i] = cv::saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
    }
 
    const int channels = image.channels();
    switch (channels)  
    {
        case 1:{
                cv::MatIterator_<uchar> it, end;
                for (it = image.begin<uchar>(), end = image.end<uchar>(); it != end; it++)
                    *it = lut[(*it)];
                break;
               }
        case 3:{ 
                cv::MatIterator_<cv::Vec3b> it, end;
                for (it = image.begin<cv::Vec3b>(), end = image.end<cv::Vec3b>(); it != end; it++)
                {
                    (*it)[0] = lut[((*it)[0])];
                    (*it)[1] = lut[((*it)[1])];
                    (*it)[2] = lut[((*it)[2])];
                }
                break;
                }
    }

}
