#include "ScaleEstimator.h"



int main(int argc, char** argv) {
    if(argc != 8)
    {
        std::cerr << std::endl << "Usage: ./Test_scale database_path sparse_map_path voc_indices_path \n"
                     << "      test_image_traj test_images_path focus_length num_images\n";
        return 1;
    }

    double focus_length = atoi(argv[6]);
    int num_images = atoi(argv[7]);

    std::cout << "\n Test image from : " << argv[4] << std::endl;
    std::cout << "   image focus length : " << focus_length << std::endl;
    std::cout << "  " << num_images << " images to test.\n";

    Ulocal::LocalizationLY *pLocalizationLY;
    pLocalizationLY = new Ulocal::LocalizationLY(argv[1], argv[2], argv[3], true, false);

    Estimator::ScaleEstimator(pLocalizationLY, argv[4], argv[5], focus_length, num_images);

    pLocalizationLY->View();

}
