#include "configuration.h"

bool is_file_exist(const std::string fileName)
{
    std::ifstream infile(fileName);
    if(!infile.good()){
        std::cout << "[ERROR] file not exist \n";
        return false;
    }
    //std::cout << fileName << " file found \n";
    return true;
}
