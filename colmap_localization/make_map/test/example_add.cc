#include "u_mapping.h"

#include "UMappingPlugins.h"

using namespace colmap;

void print(char* key, int flag)
{
    std::string key_str(key);
    std::cout << " [REGISTERED CALLBACK TEST] " << key_str << " " << flag << "\n";
}

int main(int argc, char** argv) {

    if(argc < 6)
    {
        std::cerr << std::endl << "Usage: ./Make_map_add old_work_space_path resource_path \n"
                  << " voc_1_path voc_2_path voc_3_path \n\n"
                  << " resource path must be consist of folder videos, and folder images. \n";
        return 1;
    }

    UMapping::MakeMapLY* pMakeMapLY = new UMapping::MakeMapLY();
 
    std::string pre_fix = "ABCS_2";

    pMakeMapLY->SetVocabularyPaths(argv[3], argv[4], argv[5]);

    pMakeMapLY->SetBuildParameters(true, 0);

    pMakeMapLY->register_callback((UMapping::REGISTER_CALLBACK)print);

    pMakeMapLY->ColmapMappingAdditionImages(argv[1], argv[2], pre_fix);

    return EXIT_SUCCESS;
}
