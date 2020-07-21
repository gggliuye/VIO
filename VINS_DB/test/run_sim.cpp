
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

using namespace std;
using namespace cv;
using namespace Eigen;


double numWait = 1;
// data path
string sData_path = "/home/viki/Documents/VioDeepBlue/vio_data_simulation/bin/";
string sConfig_path = "../config/simulation_config.yaml";

string traj_save_path = "/home/viki/Documents/VioDeepBlue/trajectory_sim.txt";

// the VI O/SLAM system
std::shared_ptr<System> pSystem;

void PubImuData()
{
    //double dt = 1/200;
    //double sleepTime = dt*1e9;

	string sImu_data_file = sData_path + "imu_pose_noise.txt";

	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;
    double qw, qx,qy,qz,tx,ty,tz;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampSec ;
        ssImuData >> qw >> qx >> qy >> qz >> tx >> ty >> tz;
        ssImuData >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		//cout << "Imu t: " << dStampSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << std::endl;
             //<< " other " << qw << " " << qx << " " << qy << " " << qz << " " << tx << " " << ty << " " << tz << " " << endl;
		pSystem->PubImuData(dStampSec, vGyr, vAcc);
		usleep(5000*numWait);
	}
	fsImu.close();
}

void PubImageData()
{
    int num = 0;
    //double dStampSec = 0;
    //double dt = 1/30;
    //double sleepTime = dt*1e6;

    IMG_MSG* last_features = new IMG_MSG();

	while (num < 600)
	{
        std::string keyframe_path = sData_path + "keyframe/all_points_withid_" + std::to_string(num) + ".txt";\

        //std::cout << "process file: " << keyframe_path << std::endl;

	    //pSystem->PubImageData(dStampNSec / 1e9, img);
        IMG_MSG* feature_points = pSystem->load_Feature_from_simulation(keyframe_path, last_features);

        last_features = feature_points;
         
        //std::cout << "  test:: " << feature_points->header << " " << dStampSec << std::endl;
     
        pSystem->PubImageFeatures(feature_points);

        num++;
        //dStampSec = dStampSec + 0.033333F;

		//usleep(38333);
        usleep(38333*numWait);
	}

}

int main(int argc, char **argv)
{

    // set the system configuration files (parameters)
	pSystem.reset(new System(sConfig_path));
	
    // start the backend process thread
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);

    // start the Imu data read thread
	std::thread thd_PubImuData(PubImuData);

    // start the Image read thread
	std::thread thd_PubImageData(PubImageData);
	
    // start the viewer thread
	std::thread thd_Draw(&System::Draw, pSystem);
	
    // join() : When we call this method using a thread object, it suspends the execution
    // of the calling thread until the object called finishes its execution.
	thd_PubImuData.join();
	thd_PubImageData.join();

	//thd_BackEnd.join();
	//thd_Draw.join();

    pSystem->save_Pose_asTUM(traj_save_path);

	cout << "main end... see you ..." << endl;
	return 0;
}
