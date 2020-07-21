#include "u_mapping.h"

#include "UMappingPlugins.h"

using namespace colmap;

void print(char* key, int flag)
{
    std::string key_str(key);
    std::cout << " [REGISTERED CALLBACK TEST] " << key_str << " " << flag << "\n";
}

int main(int argc, char** argv) {

    if(argc < 5)
    {
        std::cerr << std::endl << "Usage: ./Extract_image work_space_path resource_path feature_parallax_threshold laplacian_threshold\n"
                  << " resource path must be consist of folder videos, and folder images. \n";
        return 1;
    }

    std::string work_space = argv[1];
    std::string cmd = "rm -rf " + work_space + "/*";
    system(cmd.data());

    UMapping::MakeMapLY* pMakeMapLY = new UMapping::MakeMapLY();
 
    std::string pre_fix = "Extract_image_test";
    pMakeMapLY->register_callback((UMapping::REGISTER_CALLBACK)print);

    if(argc == 6){
        float resize_ratio = atof(argv[5]);
        pMakeMapLY->SetToSaveResizedImage(resize_ratio);
    }

    double feature_parallax_threshold = atof(argv[3]);
    double laplacian_threshold = atof(argv[4]);
    pMakeMapLY->SetImageExtractionParameters(feature_parallax_threshold, laplacian_threshold);
    pMakeMapLY->ExtractImages(argv[1], argv[2], pre_fix);

    return EXIT_SUCCESS;
}
