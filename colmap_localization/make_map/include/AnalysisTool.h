#ifndef ANALYSIS_TOOL_H_
#define ANALYSIS_TOOL_H_

#include <random>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "LocalizationLY.h"

namespace Ulocal{

class AnalysisTool
{
public:
    AnalysisTool(Ulocal::LocalizationLY *pLocalizationLY_){
        pLocalizationLY = pLocalizationLY_;
        resolution_direction = resolution * 7;
    }
    ~AnalysisTool(){}

    void ProjectMapIntoImage(cv::Mat &mImageShow, Eigen::Matrix4d ProjectionToPlane, Eigen::Vector2d vHeightRange, float scale_, bool bSave = false);

    void DrawViewRange(cv::Mat &mImageShow, Eigen::Vector2d vPosition, Eigen::Vector2d vDirection);

    std::string output_file = "Map.dat";

    void TestMapRead(std::string &read_file);

private:
    float resolution = 40.0;
    double resolution_direction;

    Ulocal::LocalizationLY *pLocalizationLY;


};

} // namespace
#endif
