#include <vector>
#include <string>
#include <dirent.h>
#include <random>

#include "SIFTExtractor.h"
#include "LocalLocalization.h"

#include <opencv2/highgui/highgui.hpp>

/*
./Test_video_images /home/viki/UTOPA/Server_Localization/Maps/winter_garden/SavedMap.dat /home/viki/UTOPA/Server_Localization/Maps/winter_garden_test/ /home/viki/UTOPA/Server_Localization/Maps/video_result.txt 596.1

*/


struct Trajectory{
    Trajectory(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3):
         pose_tool(p1), pose_noise(p2), pose_esti(p3) {}

    Eigen::Vector3d pose_tool;
    Eigen::Vector3d pose_noise;
    Eigen::Vector3d pose_esti;
};

void drawTraj(std::vector<Trajectory> &vTrajectory)
{
    // draw top-view 
    float min_x = 999;
    float max_x = -999;

    float min_y = 999;
    float max_y = -999;

    float min_z = 999;
    float max_z = -999;

    int N = vTrajectory.size();
    for(int i = 0; i < N ; i++){
        Eigen::Vector3d pose_i = vTrajectory[i].pose_tool;
        if(pose_i(0) > max_x) {max_x = pose_i(0);}
        if(pose_i(0) < min_x) {min_x = pose_i(0);}
        if(pose_i(1) > max_y) {max_y = pose_i(1);}
        if(pose_i(1) < min_y) {min_y = pose_i(1);}
        if(pose_i(2) > max_z) {max_z = pose_i(2);}
        if(pose_i(2) < min_z) {min_z = pose_i(2);}
        pose_i = vTrajectory[i].pose_noise;
        if(pose_i(0) > max_x) {max_x = pose_i(0);}
        if(pose_i(0) < min_x) {min_x = pose_i(0);}
        if(pose_i(1) > max_y) {max_y = pose_i(1);}
        if(pose_i(1) < min_y) {min_y = pose_i(1);}
        if(pose_i(2) > max_z) {max_z = pose_i(2);}
        if(pose_i(2) < min_z) {min_z = pose_i(2);}
        pose_i = vTrajectory[i].pose_esti;
        if(pose_i(0) > max_x) {max_x = pose_i(0);}
        if(pose_i(0) < min_x) {min_x = pose_i(0);}
        if(pose_i(1) > max_y) {max_y = pose_i(1);}
        if(pose_i(1) < min_y) {min_y = pose_i(1);}
        if(pose_i(2) > max_z) {max_z = pose_i(2);}
        if(pose_i(2) < min_z) {min_z = pose_i(2);}
    }

    // 640 * 640 is the top map
    // 640 * 240 is the side map
    cv::Mat image_show = cv::Mat::zeros(880,640,CV_8UC3) +  cv::Scalar(255, 255, 255);

    // top map : x/z map
    float max_range = std::max((max_x-min_x), (max_z-min_z)) + 0.1;
    float ratio = 640 / max_range;

    min_y = min_y - 0.1;
    float max_range_y = max_y - min_y;
    float ratio_y = 240 / max_range_y;

    for(int i = 0; i < N ; i++){
        Eigen::Vector3d pose_i = vTrajectory[i].pose_tool;
        cv::circle(image_show, cv::Point2f((pose_i(0)-min_x)*ratio, (pose_i(2)-min_z)*ratio), 5, cv::Scalar(255,0,0),2);
        cv::circle(image_show, cv::Point2f((pose_i(0)-min_x)*ratio, 640+(pose_i(1)-min_y)*ratio_y), 5, cv::Scalar(255,0,0),2);
        pose_i = vTrajectory[i].pose_noise;
        cv::circle(image_show, cv::Point2f((pose_i(0)-min_x)*ratio, (pose_i(2)-min_z)*ratio), 5, cv::Scalar(0,255,0),2);
        cv::circle(image_show, cv::Point2f((pose_i(0)-min_x)*ratio, 640+(pose_i(1)-min_y)*ratio_y), 5, cv::Scalar(0,255,0),2);
        pose_i = vTrajectory[i].pose_esti;
        cv::circle(image_show, cv::Point2f((pose_i(0)-min_x)*ratio, (pose_i(2)-min_z)*ratio), 5, cv::Scalar(0,0,255),2);
        cv::circle(image_show, cv::Point2f((pose_i(0)-min_x)*ratio, 640+(pose_i(1)-min_y)*ratio_y), 5, cv::Scalar(0,0,255),2);

        if(i == 0)
            continue;

        Eigen::Vector3d Distance = vTrajectory[i].pose_tool - vTrajectory[i-1].pose_tool;
        if(Distance.norm() > 2)
            continue;
        Eigen::Vector3d pose_j = vTrajectory[i-1].pose_tool;
        cv::line(image_show, cv::Point2f((pose_i(0)-min_x)*ratio, (pose_i(2)-min_z)*ratio), 
                             cv::Point2f((pose_j(0)-min_x)*ratio, (pose_j(2)-min_z)*ratio), cv::Scalar(255, 0, 0), 2);
        cv::line(image_show, cv::Point2f((pose_i(0)-min_x)*ratio, 640+(pose_i(1)-min_y)*ratio_y), 
                             cv::Point2f((pose_j(0)-min_x)*ratio, 640+(pose_j(1)-min_y)*ratio_y), cv::Scalar(255, 0, 0), 2);

        pose_i = vTrajectory[i].pose_noise; pose_j = vTrajectory[i-1].pose_noise;
        cv::line(image_show, cv::Point2f((pose_i(0)-min_x)*ratio, (pose_i(2)-min_z)*ratio), 
                             cv::Point2f((pose_j(0)-min_x)*ratio, (pose_j(2)-min_z)*ratio), cv::Scalar(0, 255, 0), 2);
        cv::line(image_show, cv::Point2f((pose_i(0)-min_x)*ratio, 640+(pose_i(1)-min_y)*ratio_y), 
                             cv::Point2f((pose_j(0)-min_x)*ratio, 640+(pose_j(1)-min_y)*ratio_y), cv::Scalar(0, 255, 0), 2);

        pose_i = vTrajectory[i].pose_esti; pose_j = vTrajectory[i-1].pose_esti;
        cv::line(image_show, cv::Point2f((pose_i(0)-min_x)*ratio, (pose_i(2)-min_z)*ratio), 
                             cv::Point2f((pose_j(0)-min_x)*ratio, (pose_j(2)-min_z)*ratio), cv::Scalar(0, 0, 255), 2);
        cv::line(image_show, cv::Point2f((pose_i(0)-min_x)*ratio, 640+(pose_i(1)-min_y)*ratio_y), 
                             cv::Point2f((pose_j(0)-min_x)*ratio, 640+(pose_j(1)-min_y)*ratio_y), cv::Scalar(0, 0, 255), 2);
    }
   
    cv::line(image_show, cv::Point2f(0,640), cv::Point2f(640,640), cv::Scalar(0, 0, 0), 2);

    {
        std::string text = "o groud result";
        cv::putText(image_show, text, cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 2, 8, 0);
    }
    {
        std::string text = "o noise result";
        cv::putText(image_show, text, cv::Point(10, 60), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8, 0);
    }
    {
        std::string text = "o estimated result";
        cv::putText(image_show, text, cv::Point(10, 90), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8, 0);
    }

    cv::imwrite("debug.png", image_show);

}

void ReadPose(std::string pose_images_txt, std::vector<Eigen::Vector4d> &vQvecs, std::vector<Eigen::Vector3d> &vTvecs,
            std::vector<int> &vIndices)
{
    ///////////////////// read pose estimations //////////////////////////
    std::cout << " Read pose estimations from file : " << pose_images_txt << std::endl;
    std::ifstream fin_pose(pose_images_txt, std::ios_base::in );
    if ( !fin_pose.is_open ( ) ){
        std::cout << " Cannot open image path file. " << std::endl;
    }
    
    //std::vector<Eigen::Vector4d> vQvecs;
    //std::vector<Eigen::Vector3d> vTvecs;

    vQvecs.clear();
    vTvecs.clear();
    vIndices.clear();

    Eigen::Vector4d qvec_t;
    Eigen::Vector3d tvec_t;
    int frame_idx;

    int index = 0;
    std::string str;
    char ch;
    while (!fin_pose.eof( )){
        fin_pose.get(ch);
        if( ch != ' ' && ch != '\t' && ch != '\n' ){
            str.push_back(ch);
        } else if (ch == '\n') {
            qvec_t(3) = atof(str.c_str());
            //std::cout << frame_idx << " : " << qvec_t(0) << " " << qvec_t(1) << " " << qvec_t(2) << " " << qvec_t(3) << " "
            //          << tvec_t(0) << " " << tvec_t(1) << " " << tvec_t(2) << "\n";
            vQvecs.push_back(qvec_t);
            vTvecs.push_back(tvec_t);
            vIndices.push_back(frame_idx);
            str.clear();
            index = 0;
        } else if (ch == ' '){
            //std::cout << str << "\n";
            if(index <= 0){
                frame_idx = atoi(str.c_str());
            } else if(index <= 3){
                tvec_t(index - 1) = atof(str.c_str());
            } else {
                qvec_t(index - 4) = atof(str.c_str());
            }
            str.clear();
            index++;
        } else {
            str.clear();
        }
    }

    vQvecs.resize(vQvecs.size()-1);
    vTvecs.resize(vTvecs.size()-1);
    vIndices.resize(vIndices.size()-1);

    fin_pose.close();
}

std::vector<std::string> ReadVideosFromFolder(std::string &video_folder)
{
    std::vector<std::string> video_paths;
    // read videos
    {
        video_paths.clear();
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir (video_folder.c_str())) == NULL) {
            std::cerr << "[ERROR] cannot open videos folder ! " << std::endl;
            return video_paths;
        }
        while ((ent = readdir (dir)) != NULL) {
            std::string pathvideo = video_folder + ent->d_name;
            if(pathvideo.length() - video_folder.length() < 4){
                continue;
            }
            video_paths.push_back(pathvideo);
        }
    }
    std::cout << "==> Have Read " 
              << video_paths.size() << " videos. \n\n";
    return video_paths;
}


int main(int argc, char** argv) {
    if(argc != 6)
    {
        std::cerr << std::endl << "Usage: ./Test_video_images binary_map \n"
                     << "       test_video_folder text_video_results focus_length option \n";
        return 1;
    }

    double focus_length = atof(argv[4]);
    int option_loc = atoi(argv[5]);
    std::string result_txt = argv[3];
    std::string video_folder = argv[2];
    std::string map_file = argv[1];

    std::cout << "\n==> Test videos from : " << video_folder << std::endl;
    std::cout << "==> image focus length : " << focus_length << std::endl;

    std::vector<std::string> video_paths = ReadVideosFromFolder(video_folder);

    std::vector<Eigen::Vector4d> vQvecs;
    std::vector<Eigen::Vector3d> vTvecs;
    std::vector<int> vIndices;
    ReadPose(result_txt, vQvecs, vTvecs, vIndices);
    std::cout << "==> Get " << vQvecs.size() << " poses.\n";

    BASTIAN::TicToc time_load;
    BASTIAN::LocalLocalization* pLocalLocalization = new BASTIAN::LocalLocalization(map_file);
    std::cout << "==> Load Map Time : " << time_load.Now() << std::endl;

    pLocalLocalization->SetFile("log.txt");

    //Ulocal::LocalizationLY *pLocalizationLY;
    //pLocalizationLY = new Ulocal::LocalizationLY(argv[1], argv[2], argv[3], true, true);

    int success_count = 0;
    int fail_count = 0;
    int test_count = 0;
    int interval = 30;

    double time_success = 0;
    double time_failed = 0;

    int result_idx = 0;

    std::default_random_engine e;
    std::uniform_real_distribution<double> n_tran(-0.3, 0.3);
    std::uniform_real_distribution<double> n_rot(-0.15, 0.15);

    double distance_estimated = 0;
    double distance_noise = 0;

    std::vector<Trajectory> vTrajectory;

    for(size_t i = 0; i < video_paths.size() ; i++){
        std::cout << "\n==> input new video : " << video_paths[i] << std::endl;
        cv::VideoCapture capture;
        cv::Mat frame;
        frame = capture.open(video_paths[i]);
        if(!capture.isOpened()){
            std::cout << " [ERROR] fail to open video " << video_paths[i] << "\n";
            continue;
        }

        while(capture.read(frame)) {
            BASTIAN::TicToc time_process;

            cv::Mat frame_process;
            cv::Mat imageGray;
            cv::resize(frame, frame_process, cv::Size(frame.cols/3, frame.rows/3));
            cvtColor(frame_process, imageGray, CV_RGB2GRAY);

            // find the corresponding pose estimation result
            while(vIndices[result_idx] < test_count){
                result_idx++;
            }

            if(vIndices[result_idx] == test_count){
                std::cout << "\n==> Load pose of " << test_count << "th frame.\n";
 
                // noise pose translation
                Eigen::Vector3d noise_t(n_tran(e), n_tran(e), n_tran(e));

                // noise pose rotation 
                Eigen::Vector3d ea0(n_rot(e),n_rot(e),n_rot(e));
                Eigen::Matrix3d R_noise;
                R_noise = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());

                std::cout << "==> Noise Pose (euler, t) : " << ea0.transpose() << ", " << noise_t.transpose() << std::endl;
                std::cout << "==> Tool Map Pose (q,t) : " << vQvecs[result_idx].transpose() << ", " 
                          << vTvecs[result_idx].transpose() << std::endl;

                Eigen::Vector4d qvec = vQvecs[result_idx];
                Eigen::Quaterniond q_ori(qvec(0), qvec(1), qvec(2), qvec(3));
                Eigen::Quaterniond q_noised(R_noise * q_ori.matrix());

                qvec << q_noised.w(), q_noised.x(), q_noised.y(), q_noised.z();
                Eigen::Vector3d tvec = vTvecs[result_idx] + noise_t;
                bool ret_loc = false;
                if(option_loc == 1){
                    ret_loc = pLocalLocalization->LocalizeImage(imageGray, focus_length, qvec, tvec);
                } else if(option_loc == 2) {
                    ret_loc = pLocalLocalization->LocalizeImageFLANN(imageGray, focus_length, qvec, tvec);
                }
                if(ret_loc){
                    std::cout << "==> Estimated Pose (q,t) : " << qvec.transpose() << ", " 
                          << tvec.transpose() << std::endl;

                    float distance_e = (tvec - vTvecs[result_idx]).norm();
                    // calculate the distance of translation
                    std::cout << "==> distance noise : " << noise_t.norm() << ", distance estimated : " 
                              << distance_e << std::endl;

                    //if(distance_e > 1.5){ fail_count++;continue;}

                    distance_estimated += distance_e;
                    distance_noise += noise_t.norm();
                    success_count++;
                    double time_process_t = time_process.Now();
                    time_success += time_process_t;

                    Trajectory traj(vTvecs[result_idx], vTvecs[result_idx]+noise_t, tvec);
                    vTrajectory.push_back(traj);

                    std::vector<BASTIAN::FeatureKeypoint> vOutputKeypoints = pLocalLocalization->pLastFrame->vFeatureKeypoints;
                    std::vector<BASTIAN::PointMatches> vPointMatches = pLocalLocalization->pLastFrame->vPointMatches;
                    int count = 0;
                    for(size_t i = 0 ; i < vOutputKeypoints.size(); i ++){
                        if(vPointMatches[i].flag){
                            count ++;
                            cv::circle(frame_process, cv::Point2f(vOutputKeypoints[i].x, vOutputKeypoints[i].y), 3, cv::Scalar(255,0,0),-1);
                        } else
                            cv::circle(frame_process, cv::Point2f(vOutputKeypoints[i].x, vOutputKeypoints[i].y), 3, cv::Scalar(0,255,0),0);
                    }
                    std::string text = std::to_string(count) + ", " + std::to_string(distance_e);
                    cv::putText(frame_process, text, cv::Point(20, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8, 0);

                    cv::imshow("Process Frame",frame_process);
                    cv::waitKey(1);

                    if(pLocalLocalization->b_debug){
                        pLocalLocalization->runtimefile << " " << time_process_t << " " << noise_t.norm() << " " << distance_e << std::endl;
                    }

                } else {
                    fail_count++;
                    time_failed += time_process.Now();
                }
            }
       
            test_count++;
            int interval_count = 0;
            while(capture.read(frame) && interval_count < interval){
                // skip this frame
                interval_count++;
            }
        }
    }

    if(pLocalLocalization->b_debug){
        pLocalLocalization->runtimefile.close();
    }

    drawTraj(vTrajectory);

    float success_rate = (float)success_count / (float)(success_count + fail_count);
    std::cout << "\n==> Success rate " << success_rate << " [ " << success_count << " / " << (success_count + fail_count) << " ]"
              << std::endl;
    std::cout << "==> Success average runtime : " << (time_success/success_count) << std::endl;

    std::cout << "==> Average noise translation distance is " << (distance_noise/success_count)
              << ", Average estimated translation distance is " << (distance_estimated/success_count) << std::endl;

    return EXIT_SUCCESS;
}
