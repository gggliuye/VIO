/**
*    plugins of Ulocalization for Server Use
*
*    LIU YE @ moon light
*
*/


#ifndef ULOCALIZATION_BETA_PLUGINS_H
#define ULOCALIZATION_BETA_PLUGINS_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "LocalizationLY.h"


extern "C" int Internal_Clear_Map_Beta();

extern "C" int Internal_Map_Length_Beta();

extern "C" int Internal_Check_Key_Existance_Beta(const int key);

extern "C" int Internal_Init_Ulocalization_Map_Beta(const char *database_path, const char* reconstruction_path,
                                const char* vocIndex_path, const int key);

extern "C" int Internal_Destroy_Ulocalization_Map_Beta(const int key);

extern "C" float* Internal_Track_Ulocalization_Map_Beta(unsigned char* inputImage, int bufferLength, 
                     double focus_length, int deviceType, 
                     float* init_pose, bool bUseInitGuess,
                     const int key);




extern "C" float* EigenPoseToFloatArray(Eigen::Vector4d &qvec, Eigen::Vector3d &tvec, bool ifSuccess);
extern "C" void FloatArrayToEigenPose(Eigen::Vector4d &qvec, Eigen::Vector3d &tvec, float* mModelview_matrix);

#endif // ULOCALIZATION_BETA_PLUGINS_H
