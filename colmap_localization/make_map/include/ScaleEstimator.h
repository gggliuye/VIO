#ifndef SCALE_ESTIAMTOR_LY_H_
#define SCALE_ESTIAMTOR_LY_H_


#include <random>
#include <string>
#include <iostream>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "LocalizationLY.h"



class Estimator
{

public : 
    static double ScaleEstimator(Ulocal::LocalizationLY *pLocalizationLY, 
                 const std::string &arcore_traj_file, const std::string &arcore_image_path,
                 double focus_length, int num_images);

    static double pose_estimation_SVD(
	         const std::vector<Eigen::Vector3d>& pts1,
	         const std::vector<Eigen::Vector3d>& pts2,
	         Eigen::Matrix3d& R, Eigen::Vector3d& t);

};

#endif 
