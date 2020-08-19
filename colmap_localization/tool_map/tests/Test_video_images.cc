#include <vector>
#include <string>
#include <dirent.h>

#include "LocalizationLY.h"

using namespace colmap;

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
    double focus_length = 500;
    if(argc != 6)
    {
        //std::cout << argc << std::endl;
        if(argc == 5){
            focus_length = 500;
        } else {
            std::cerr << std::endl << "Usage: ./Test_video_images database_path sparse_map_path voc_indices_path \n"
                     << "       test_video_folder focus_length \n";
            return 1;
        }
    } else {
        focus_length = atoi(argv[5]);
    }

    std::string video_folder = argv[4];

    std::cout << "\n==> Test videos from : " << argv[4] << std::endl;
    std::cout << "==> image focus length : " << focus_length << std::endl;

    std::vector<std::string> video_paths = ReadVideosFromFolder(video_folder);

    Ulocal::LocalizationLY *pLocalizationLY;
    pLocalizationLY = new Ulocal::LocalizationLY(argv[1], argv[2], argv[3], true, true);

    int success_count = 0;
    int test_count = 0;
    int interval = 30;

    double time_success = 0;
    double time_failed = 0;

    std::string work_path = argv[2];
    std::string runtimeFilename = work_path + "/video_result.txt";
    std::cout << " [SAVE OUTPUT] save output to " << runtimeFilename << std::endl;
    std::ofstream runtimefile;
    runtimefile.open(runtimeFilename.c_str());
    runtimefile << std::fixed;

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
            TicToc time_process;

            cv::Mat frame_process;
            cv::resize(frame, frame_process, cv::Size(frame.cols/3, frame.rows/3));

            cv::imshow("Process Frame",frame_process);
            cv::waitKey(10);

            Eigen::Vector4d qvec;
            Eigen::Vector3d tvec;
            if(pLocalizationLY->LocalizeImage(frame_process, focus_length, qvec, tvec)){
                success_count++;
                time_success += time_process.Now();

                runtimefile << test_count << " " << tvec(0) << " " << tvec(1) << " " << tvec(2) << " ";
                runtimefile << qvec(0) << " " << qvec(1) << " " << qvec(2) << " " << qvec(3) << "\n";
            } else {
                time_failed += time_process.Now();
            }
            test_count++;

            int interval_count = 0;
            while(capture.read(frame) && interval_count < interval){
                // skip this frame
                interval_count++;
            }
        }
    }

    runtimefile.close();

    float success_rate = (float)success_count / (float)test_count;
    std::cout << StringPrintf("==> Success rate %f [ %d / %d ]", success_rate, success_count, test_count) 
              << std::endl;

    std::cout << StringPrintf("==> Average time,  success: %f, fail : %f", time_success/success_count, time_failed/(test_count - success_count)) 
              << std::endl;

    pLocalizationLY->View();

    return EXIT_SUCCESS;
}
