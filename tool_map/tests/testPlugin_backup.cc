#include <thread>

#include "test_utils.h"
#include "UlocalizationPlugins.h"
#include "UlocalizationPluginsBeta.h"


// include opencv libraries, only used here in the example
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

// file system C++ need C++17
//#include <filesystem>
#include <dirent.h>

//void testPlugin_version_alpha();

void testPlugin_version_beta(std::string path, std::string map_path, std::string vocIndex_path);

void TestImageToProcessKeyID_1();

void TestImageToProcessKeyID_2();

void TestImageToProcessKeyID(std::vector<std::string> image_paths, int key, double focus_length);

std::vector<std::string> image_paths;
double focus_length;

int main(int argc, char** argv) {

    if(argc != 7)
    {
        std::cerr << std::endl << "Usage: ./test_plugin database_path sparse_map_path voc_indices_path \n"
                     << "          path_images_txt_path pose_images_txt_path focus_length \n";
        return 1;
    }  

    testPlugin_version_beta(argv[1], argv[2], argv[3]);

    focus_length = atoi(argv[6]);

    ///////////////////// read image paths ///////////////////////////
    std::cout << " Read image paths from file : " << argv[4] << std::endl;

    std::string path_images_txt = argv[4];
    std::ifstream fin(path_images_txt, std::ios_base::in );
    if ( !fin.is_open ( ) ){
        std::cout << " Cannot open image path file. " << std::endl;
    }
    //std::vector<std::string> image_paths;
    std::string str;
    char ch;
    while (!fin.eof( )){
        fin.get(ch);
        if( ch != ' ' && ch != '\t' && ch != '\n' ){
            str.push_back(ch);
        } else {
            if(str.length() > 1){
                image_paths.push_back(str);
                //std::cout << str << std::endl;
                str.clear();
            }
        }
    }

    ///////////////////// read pose estimations //////////////////////////
    std::cout << " Read pose estimations from file : " << argv[5] << std::endl;
    std::string pose_images_txt = argv[5];
    std::ifstream fin_pose(pose_images_txt, std::ios_base::in );
    if ( !fin_pose.is_open ( ) ){
        std::cout << " Cannot open image path file. " << std::endl;
    }
    
    std::vector<Eigen::Vector4d> vQvecs;
    std::vector<Eigen::Vector3d> vTvecs;

    Eigen::Vector4d qvec_t;
    Eigen::Vector3d tvec_t;
    int index = 0;
    while (!fin_pose.eof( )){
        fin_pose.get(ch);
        if( ch != ' ' && ch != '\t' && ch != '\n' ){
            str.push_back(ch);
        } else if (ch == '\n') {
            //std::cout << qvec_t(0) << " " << qvec_t(1) << " " << qvec_t(2) << " " << qvec_t(3) << " "
            //          << tvec_t(0) << " " << tvec_t(1) << " " << tvec_t(2) << "\n";
            vQvecs.push_back(qvec_t);
            vTvecs.push_back(tvec_t);
            str.clear();
            index = 0;
        } else if (ch == ' '){
            //std::cout << str << "\n";
            if(index <= 3){
                qvec_t(index) = atof(str.c_str());
            } else if(index <= 6){
                tvec_t(index-4) = atof(str.c_str());
            }
            str.clear();
            index++;
        } else {
            str.clear();
        }
    }

    std::thread thd_1(TestImageToProcessKeyID_1);
    std::thread thd_2(TestImageToProcessKeyID_2);
	
    thd_1.join();
    thd_2.join();

    Internal_Clear_Map_Beta();

    return EXIT_SUCCESS;
}

void TestImageToProcessKeyID_1()
{
    std::cout << " ---- [START THREAD 1] ---- " << std::endl;
    TestImageToProcessKeyID(image_paths, 1, focus_length);
}

void TestImageToProcessKeyID_2()
{
    std::cout << " ---- [START THREAD 2] ---- " << std::endl;
    TestImageToProcessKeyID(image_paths, 2, focus_length);
}

void TestImageToProcessKeyID(std::vector<std::string> image_paths, int key, double focus_length)
{
    for(size_t i = 0; i < image_paths.size(); i++){
        cv::Mat image = cv::imread(image_paths[i]);
        cv::cvtColor(image,image,cv::COLOR_BGR2GRAY);

        std::vector<unsigned char> inImage;
        cv::imencode(".jpg",image, inImage);
        size_t bufferLength = inImage.size();
        unsigned char *msgImage = new unsigned char[bufferLength];
        for(int i=0; i < bufferLength; i++){
            msgImage[i] = inImage[i];
        }

        float* pose_1 = Internal_Track_Ulocalization_Map_Beta(
                         msgImage, bufferLength,
                         focus_length, SHADOW_CREATOR,
                         NULL, false, key);
        
        float* pose_2 = Internal_Track_Ulocalization_Map_Beta(
                         msgImage, bufferLength,
                         focus_length, SHADOW_CREATOR,
                         pose_1, true, key);
        
    }
}

void testPlugin_version_beta(std::string path, std::string map_path, std::string vocIndex_path)
{
    Internal_Init_Ulocalization_Map_Beta(path.c_str(), map_path.c_str(), vocIndex_path.c_str(), 1);

    Internal_Init_Ulocalization_Map_Beta(path.c_str(), map_path.c_str(), vocIndex_path.c_str(), 2);

    Internal_Init_Ulocalization_Map_Beta(path.c_str(), map_path.c_str(), vocIndex_path.c_str(), 1);

    std::cout << " - Map size is : " << Internal_Map_Length_Beta() << std::endl;

    std::cout << " - Whether map 2 exist : " <<  Internal_Check_Key_Existance_Beta(2) << std::endl; 

    std::cout << " - Whether map 3 exist : " <<  Internal_Check_Key_Existance_Beta(3) << std::endl;

    std::cout << std::endl;
}

/*
void testPlugin_version_alpha()
{
    Internal_Init_Ulocalization_Map(path.c_str(), map_path.c_str(), vocIndex_path.c_str(), 1);

    Internal_Init_Ulocalization_Map(path.c_str(), map_path.c_str(), vocIndex_path.c_str(), 2);

    Internal_Init_Ulocalization_Map(path.c_str(), map_path.c_str(), vocIndex_path.c_str(), 1);

    std::cout << "Map size is : " << Internal_Map_Length() << std::endl;

    std::cout << "Whether map 2 exist : " <<  Internal_Check_Key_Existance(2) << std::endl; 

    std::cout << "Whether map 3 exist : " <<  Internal_Check_Key_Existance(3) << std::endl;

    Internal_Clear_Map();
}
*/


