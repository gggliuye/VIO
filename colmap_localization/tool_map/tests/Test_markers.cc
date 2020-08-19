#include <vector>
#include <string>
#include <dirent.h>

#include "LocalizationLY.h"

/*

./make_index /home/viki/UTOPA/Server_Localization/Maps/pop_art/database.db /home/viki/UTOPA/Server_Localization/Maps/pop_art/sparse/ /home/viki/UTOPA/Server_Localization/Maps/pop_art/VocIndex.bin /home/viki/UTOPA/Server_Localization/vocabs/vocab_tree_flickr100K_words32K.bin

./Test_markers /home/viki/UTOPA/Server_Localization/Maps/pop_art/database.db /home/viki/UTOPA/Server_Localization/Maps/pop_art/sparse/ /home/viki/UTOPA/Server_Localization/Maps/pop_art/VocIndex.bin
*/

using namespace colmap;

void FindMarkers(Ulocal::LocalizationLY *pLocalizationLY)
{
    int count = 0;
    for (const auto idx : pLocalizationLY->framesIds ){
        if(!pLocalizationLY->database->ExistsImage(idx)){
            continue;
        }

        std::string imagePath = pLocalizationLY->reconstruction->Image(idx).Name();
        std::string folderName(imagePath, 0, 8);

        if((folderName.compare("marker_1") == 0)){
            std::cout << "Image path : " << imagePath << std::endl;
        }

        if((folderName.compare("marker_2") == 0)){
            std::cout << "Image path : " << imagePath << std::endl;
        }

        if((folderName.compare("marker_3") == 0)){
            std::cout << "Image path : " << imagePath << std::endl;
        }

        if((folderName.compare("marker_4") == 0)){
            std::cout << "Image path : " << imagePath << std::endl;
        }
    }
}


int main(int argc, char** argv) {
    if(argc != 4)
    {
        std::cerr << std::endl << "Usage: ./Test_markers database_path sparse_map_path voc_indices_path \n";
        return 1;
    }

    Ulocal::LocalizationLY *pLocalizationLY;
    pLocalizationLY = new Ulocal::LocalizationLY(argv[1], argv[2], argv[3], true, true);

    FindMarkers(pLocalizationLY);


    pLocalizationLY->View();

    return EXIT_SUCCESS;
}
