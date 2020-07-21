#pragma once

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>

#include <fstream>
#include <condition_variable>

// #include <cv.h>
// #include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "feature_tracker.h"

#include <pangolin/pangolin.h>

    //imu for vio
    struct IMU_MSG
    {
        double header;
        Eigen::Vector3d linear_acceleration;
        Eigen::Vector3d angular_velocity;
    };
    typedef std::shared_ptr<IMU_MSG const> ImuConstPtr;

    //image for vio    
    struct IMG_MSG {
        double header;
        vector<Vector3d> points;
        vector<int> id_of_point;
        vector<float> u_of_point;
        vector<float> v_of_point;
        vector<float> velocity_x_of_point;
        vector<float> velocity_y_of_point;
    };
    typedef std::shared_ptr <IMG_MSG const > ImgConstPtr;
    
class System
{
public:
    System(std::string sConfig_files);

    ~System();

    // push in real image and IMU data
    void PubImageData(double dStampSec, cv::Mat &img);

    void PubImuData(double dStampSec, const Eigen::Vector3d &vGyr, 
        const Eigen::Vector3d &vAcc);

    // push in simulation image and IMU data, generated from "vio_data_simulation" project
    void PubImageFeatures(IMG_MSG*  feature_points_);

    IMG_MSG* load_Feature_from_simulation(std::string keyframe_path, IMG_MSG* last_features);

    //void PubImuDataSim(double dStampSec, const Eigen::Vector3d &vGyr, 
    //    const Eigen::Vector3d &vAcc);

    // thread: visual-inertial odometry
    void ProcessBackEnd();
    void Draw();
   
    // save trajectory
    void save_Pose_asTUM(std::string filename);
 
private:
    bool verbose = false;

    //feature tracker
    std::vector<uchar> r_status;
    std::vector<float> r_err;
    // std::queue<ImageConstPtr> img_buf;

    // ros::Publisher pub_img, pub_match;
    // ros::Publisher pub_restart;

    FeatureTracker trackerData[NUM_OF_CAM];
    double first_image_time;
    int pub_count = 1;
    bool first_image_flag = true;
    double last_image_time = 0;
    bool init_pub = 0;

    //estimator
    Estimator estimator;

    std::condition_variable con;
    double current_time = -1;
    std::queue<ImuConstPtr> imu_buf;
    std::queue<ImgConstPtr> feature_buf;
    // std::queue<PointCloudConstPtr> relo_buf;
    int sum_of_wait = 0;

    std::mutex m_buf;
    std::mutex m_state;
    std::mutex i_buf;
    std::mutex m_estimator;

    double latest_time;
    Eigen::Vector3d tmp_P;
    Eigen::Quaterniond tmp_Q;
    Eigen::Vector3d tmp_V;
    Eigen::Vector3d tmp_Ba;
    Eigen::Vector3d tmp_Bg;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;
    bool init_feature = 0;
    bool init_imu = 1;
    double last_imu_t = -1;

    std::ofstream ofs_pose;

    std::vector<Eigen::Vector3d> vPath_to_draw;                 // history path of body(IMU)

    std::vector<double> corresponding_timestamps;
    std::vector<Eigen::Matrix<double, 3, 4>> keyframe_history;  // record all the past keyframes

    bool bStart_backend;
    

private:

    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> getMeasurements();

    Vector2d FindVelocity(IMG_MSG* last_features, int idx, double u, double v, double dStampSec);

private:
    // functions for visualization

    float mCameraSize = 0.1;
    float mCameraLineWidth = 1;

    void DrawCamera();

    void DrawKeyframe(Eigen::Matrix3d matrix_t, Eigen::Vector3d pVector_t);
    void DrawKeyframe(Eigen::Matrix<double, 3, 4> Twc);

    void GetOpenGLCameraMatrix(Eigen::Matrix3d matrix, Eigen::Vector3d pVector, 
                           pangolin::OpenGlMatrix &M, bool if_inv);

    void GetOpenGLMatrixCamera(pangolin::OpenGlMatrix &M, Eigen::Matrix<double, 3, 4> Twc);
};
