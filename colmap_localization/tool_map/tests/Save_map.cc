#include <vector>
#include <string>
#include <dirent.h>

#include "MapSaver.h"

/*
./Save_map /home/viki/UTOPA/Server_Localization/Maps/winter_garden/database.db /home/viki/UTOPA/Server_Localization/Maps/winter_garden/dense/sparse/ /home/viki/UTOPA/Server_Localization/Maps/winter_garden/SavedMap.dat


./Save_map /home/viki/UTOPA/RealSense/Data_L515/database.db /home/viki/UTOPA/RealSense/Data_L515/colmap_dense/sparse/ /home/viki/UTOPA/RealSense/Data_L515/SavedMap.dat /home/viki/UTOPA/RealSense/Data_L515/keyframes.txt
*/

using namespace colmap;

int main(int argc, char** argv) {

    if(argc != 5)
    {
        std::cerr << std::endl << "Usage: ./Save_map database_path sparse_map_path\n"
                     << "       save_path save_path_txt \n";
    }

    std::string output_file = argv[3];

    BASTIAN::MapSaver *pMapSaver;
    pMapSaver = new BASTIAN::MapSaver(argv[1], argv[2], true);

    pMapSaver->SaveMap(output_file);

    std::string output_txt = argv[4];
    pMapSaver->SaveKeyframesTxt(output_txt);

    pMapSaver->LoadMap(output_file);


    return EXIT_SUCCESS;
}
