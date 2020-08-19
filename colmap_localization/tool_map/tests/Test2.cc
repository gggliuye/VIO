#include "LocalizationLY.h"

// file system C++ need C++17
//#include <filesystem>
#include <dirent.h>


using namespace colmap;

int main(int argc, char** argv) {
    if(argc != 6)
    {
        std::cerr << std::endl << "Usage: ./Test2 database_path sparse_map_path voc_indices_path \n"
                     << "       test_images_path focus_length \n";
        return 1;
    }

    double focus_length = atoi(argv[5]);

    std::cout << "\n Test image from : " << argv[4] << std::endl;
    std::cout << "   image focus length : " << focus_length << std::endl;

    Ulocal::LocalizationLY *pLocalizationLY;
    pLocalizationLY = new Ulocal::LocalizationLY(argv[1], argv[2], argv[3], true, true);

    std::string test_images_path = argv[4];

    std::string runtimeFilename = std::string(argv[2]) + "success_images.txt";
    std::cout << " [SUCCESS IMAGES] save output to " << runtimeFilename << std::endl;
    std::ofstream runtimefile;
    runtimefile.open(runtimeFilename.c_str());
    runtimefile << std::fixed;

    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (test_images_path.c_str())) == NULL) {
        std::cout << "[ERROR] cannot open folder ! " << std::endl;
        return EXIT_FAILURE;
    }

    while ((ent = readdir (dir)) != NULL) {
        std::cout << " [START] input new image : " << ent->d_name << std::endl;
        std::string pathimg = test_images_path + ent->d_name;
 
        if(pathimg.length() - test_images_path.length() < 4){
            continue;
        }

        Eigen::Vector4d qvec;
        Eigen::Vector3d tvec;
        cv::Mat image = cv::imread(pathimg);

        if(pLocalizationLY->LocalizeImage(image, focus_length, qvec, tvec)){
            runtimefile << pathimg << std::endl;
        }
    }


    pLocalizationLY->View();

    return EXIT_SUCCESS;
}
