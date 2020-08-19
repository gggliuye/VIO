#include "LocalizationLY.h"

// file system C++ need C++17
//#include <filesystem>
#include <dirent.h>


using namespace colmap;

int main(int argc, char** argv) {
    if(argc != 7)
    {
        std::cerr << std::endl << "Usage: ./Test_arcore database_path sparse_map_path voc_indices_path \n"
                     << "       test_images_path focus_length num_images\n";
        return 1;
    }

    double focus_length = atoi(argv[5]);
    int num_images = atoi(argv[6]);

    std::cout << "\n Test image from : " << argv[4] << std::endl;
    std::cout << "   image focus length : " << focus_length << std::endl;
    std::cout << "  " << num_images << " images to test.\n";

    Ulocal::LocalizationLY *pLocalizationLY;
    pLocalizationLY = new Ulocal::LocalizationLY(argv[1], argv[2], argv[3], true, false);

    std::string test_images_path = argv[4];

    std::string runtimeFilename = std::string(argv[2]) + "success_images.txt";
    std::cout << " [SUCCESS IMAGES] save output to " << runtimeFilename << std::endl;
    std::ofstream runtimefile;
    runtimefile.open(runtimeFilename.c_str());
    runtimefile << std::fixed;

    for(int i = 1; i < num_images + 1 ; i++){
        std::string pathimg = test_images_path + std::to_string(i) + ".png";
        Eigen::Vector4d qvec;
        Eigen::Vector3d tvec;
        cv::Mat image = cv::imread(pathimg);
        if(pLocalizationLY->LocalizeImage(image, focus_length, qvec, tvec)){
            runtimefile << i << " " << tvec(0) << " " << tvec(1) << " " << tvec(2) << " "
                        << qvec(0) << " " << qvec(1) << " " << qvec(2) << " " << qvec(3) << std::endl;
        }
    }

    pLocalizationLY->View();

    return EXIT_SUCCESS;
}
