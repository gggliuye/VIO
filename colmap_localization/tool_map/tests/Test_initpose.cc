#include "LocalizationLY.h"

// file system C++ need C++17
//#include <filesystem>
#include <dirent.h>


using namespace colmap;

int main(int argc, char** argv) {
    if(argc != 7)
    {
        std::cerr << std::endl << "Usage: ./Test2 database_path sparse_map_path voc_indices_path \n"
                     << "       path_images_txt_path pose_images_txt_path focus_length \n";
        return 1;
    }

    double focus_length = atoi(argv[6]);

    std::cout << "     image focus length : " << focus_length << std::endl;

    Ulocal::LocalizationLY *pLocalizationLY;
    pLocalizationLY = new Ulocal::LocalizationLY(argv[1], argv[2], argv[3], true, true);


    ///////////////////// read image paths ///////////////////////////
    std::cout << " Read image paths from file : " << argv[4] << std::endl;

    std::string path_images_txt = argv[4];
    std::ifstream fin(path_images_txt, std::ios_base::in );
    if ( !fin.is_open ( ) ){
        std::cout << " Cannot open image path file. " << std::endl;
    }
    std::vector<std::string> image_paths;
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

    int count_total = image_paths.size();
    int count_succ = 0;
    std::cout << std::endl;
    for(size_t i = 0 ; i < image_paths.size() ; i++){
        std::cout << " [START] input new image : " << image_paths[i] << "\n";
        //std::cout << " [INIT POSE] : " << vQvecs[i].transpose() << " " << vTvecs[i].transpose() << "\n";

        cv::Mat image = cv::imread(image_paths[i]);
        if(pLocalizationLY->LocalizeImage(image, focus_length, vQvecs[i], vTvecs[i], true)){
            count_succ++;
        }
    }

    std::cout << "FINAL : success ratio is : " << count_succ << "/" << count_total << std::endl;

    pLocalizationLY->View();

    return EXIT_SUCCESS;
}
