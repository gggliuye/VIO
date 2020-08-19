/**
*    plugins of Ulocalization for Server Use
*
*    LIU YE @ moon light
*
*
*/


#include "UMappingPlugins.h"

//UMapping::MakeMapLY* pMakeMapLY;

bool path_set = false;
std::string voc_path_1;
std::string voc_path_2;
std::string voc_path_3;

extern "C" int Internal_Set_Vocabulary_Paths(const char *temp_voc_path1, const char *temp_voc_path2, const char *temp_voc_path3)
{
    std::string voc_path_1_(temp_voc_path1);
    std::string voc_path_2_(temp_voc_path2);
    std::string voc_path_3_(temp_voc_path3);

    voc_path_1 = voc_path_1_;
    voc_path_2 = voc_path_2_;
    voc_path_3 = voc_path_3_;

    path_set = true;
    return 0;
}

extern "C" int Internal_Start_Map(const char* work_space, const char* resource_path, 
                     bool build_dense, int quality, const char* key_, UMapping::REGISTER_CALLBACK P)
{ 
    UMapping::MakeMapLY* pMakeMapLY;

    pMakeMapLY = new UMapping::MakeMapLY();
    std::cout << " Regieter a new callback function. \n";
    pMakeMapLY->register_callback(P);

    if(path_set){
        std::cout << std::endl << " Internal_Set_Vocabulary_Paths " << voc_path_1 << std::endl;
        std::cout << std::endl << " Internal_Set_Vocabulary_Paths " << voc_path_2 << std::endl;
        std::cout << std::endl << " Internal_Set_Vocabulary_Paths " << voc_path_3 << std::endl;
        pMakeMapLY->SetVocabularyPaths(voc_path_1, voc_path_2, voc_path_3);
    }

    std::cout << voc_path_1 << "\n";
    std::cout << voc_path_2 << "\n";
    std::cout << voc_path_3 << "\n";

    pMakeMapLY->SetBuildParameters(build_dense, quality);

    std::string work_space_(work_space); 
    std::string resource_path_(resource_path);
    std::string pre_fix(key_);

    {
        std::string cmd = "rm -rf " + work_space_ + "/*";
        system(cmd.data());
    }

    std::cout << std::endl << " Build Map " << pre_fix << std::endl;

    pMakeMapLY->ColmapMapping(work_space_, resource_path_, pre_fix);

    return 0;
}

extern "C" int Internal_Addition_Images(const char* work_space, const char* resource_path, 
                     bool build_dense, int quality, const char* key_, UMapping::REGISTER_CALLBACK P)
{
    UMapping::MakeMapLY* pMakeMapLY;

    pMakeMapLY = new UMapping::MakeMapLY();
    std::cout << " Regieter a new callback function. \n";
    pMakeMapLY->register_callback(P);

    if(path_set){
        pMakeMapLY->SetVocabularyPaths(voc_path_1, voc_path_2, voc_path_3);
    }

    pMakeMapLY->SetBuildParameters(build_dense, quality);

    std::string work_space_(work_space); 
    std::string resource_path_(resource_path);
    std::string pre_fix(key_);

    std::cout << std::endl << " Build Map " << pre_fix << std::endl;

    pMakeMapLY->ColmapMappingAdditionImages(work_space_, resource_path_, pre_fix);

    return 0;
}

extern "C" int Internal_Register_Callback(UMapping::REGISTER_CALLBACK P)
{
/*
    if(pMakeMapLY){
        std::cout << " Regieter a new callback function. \n";
        pMakeMapLY->register_callback(P);
        return 0;
    }
*/
    return -1;
}

extern "C" int Internal_Get_Mapping_State()
{
/*
    if(pMakeMapLY){
        return pMakeMapLY->GetMappingState();
    }
*/
    return -1;
}
