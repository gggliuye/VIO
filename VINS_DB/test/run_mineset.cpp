
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"

//214112.977011

using namespace std;
using namespace cv;
using namespace Eigen;

const double nDelayTimesImu = 10000*0.1;
const double nDelayTimesImage = 100000*0.1;

string sData_path = "/home/viki/Documents/VioDeepBlue/datasets/images/";
string sConfig_path = "../config/meta20_config.yaml";

string sImu_data_file = sData_path + "IMU.txt";
string sImage_file = sData_path + "images.txt";

string traj_save_path = "/home/viki/Documents/VioDeepBlue/results/euroc/trajectory_sim.txt";

std::shared_ptr<System> pSystem;

void PubImuData()
{
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampNSec >> vAcc.x() >> vAcc.y() >> vAcc.z() >> vGyr.x() >> vGyr.y() >> vGyr.z();
		
		pSystem->PubImuData(dStampNSec, vGyr, vAcc);
		usleep(nDelayTimesImu);
	}
	fsImu.close();
}

void PubImageData()
{
	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;
	
	// cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
                sImgFileName.erase(sImgFileName.begin(), sImgFileName.begin()+36);
		string imagePath = sData_path + sImgFileName;

		Mat img = imread(imagePath.c_str(), 0);
		if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}
		pSystem->PubImageData(dStampNSec, img);
		// cv::imshow("SOURCE IMAGE", img);
		// cv::waitKey(0);
		usleep(nDelayTimesImage);
	}
	fsImage.close();
}

int main(int argc, char **argv)
{
	pSystem.reset(new System(sConfig_path));
	
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);
		
	std::thread thd_PubImuData(PubImuData);

	std::thread thd_PubImageData(PubImageData);
	
	std::thread thd_Draw(&System::Draw, pSystem);
	
	thd_PubImuData.join();
	thd_PubImageData.join();

	thd_BackEnd.join();
	thd_Draw.join();

        pSystem->save_Pose_asTUM(traj_save_path);

	cout << "main end... see you ..." << endl;
	return 0;
}
