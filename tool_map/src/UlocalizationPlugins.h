/**
*    plugins of Ulocalization for Server Use
*
*    LIU YE @ moon light
*
*/


#ifndef ULOCALIZATION_PLUGINS_H
#define ULOCALIZATION_PLUGINS_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "test_utils.h"
#include "configuration.h"

extern "C" int Internal_Clear_Map();

extern "C" int Internal_Map_Length();

extern "C" int Internal_Check_Key_Existance(const int key);

extern "C" float* CvMatToFloatArray(cv::Mat cvMatInput, bool ifSuccess);

extern "C" int Internal_Init_Ulocalization_Map(const char *database_path, const char* reconstruction_path,
                                const char* vocIndex_path, const int key);

extern "C" int Internal_Destroy_Ulocalization_Map(const int key);

extern "C" float* Internal_Track_Ulocalization_Map(unsigned char* inputImage, int bufferLength, double focus_length, int deviceType, const int key);

#endif // ULOCALIZATION_PLUGINS_H
