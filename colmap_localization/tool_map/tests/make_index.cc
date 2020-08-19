#include "test_utils.h"
#include "UlocalizationPlugins.h"

// include opencv libraries, only used here in the example
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

// file system C++ need C++17
//#include <filesystem>
#include <dirent.h>

using namespace colmap;

//std::string tree_path = "/home/utopa/Maps/vocab_tree_flickr100K_words32K.bin";

// winter
//std::string vocIndex_path = "/home/utopa/Maps/winter_update/VocIndexTest.bin";
//std::string path = "/home/utopa/Maps/winter_update/database.db";
//std::string map_path = "/home/utopa/Maps/winter_update/sparse/";

void testFileImages(const std::string &database_path, const std::string &sparse_map_path, 
          const std::string &voc_indices_path, const std::string &vocab_path);

int main(int argc, char** argv) {
  if(argc != 5){
    std::cout << " USAGE: ./make_index database_path sparse_map_path voc_indices_path vocab_path " << std::endl;
    return 0;
  }

  testFileImages(argv[1], argv[2], argv[3], argv[4]);

  return EXIT_SUCCESS;
}

void testFileImages(const std::string &database_path, const std::string &sparse_map_path, 
          const std::string &voc_indices_path, const std::string &vocab_path)
{
    std::cout << " -- Make Index for server localization -- " << std::endl << std::endl;

    // load the prebuilt colmap map file
    Ulocal::Ulocalization ulocalization(database_path, sparse_map_path);

    //ulocalization.PrintCameraInfo();
    ulocalization.SavePLYCLoud(sparse_map_path+"kexuecheng.ply");

    // whether to remake the vocabulary
    //ulocalization.MakeVocTreeIndexForFloor(tree_path, vocIndex_path, floor);
    ulocalization.MakeVocTreeIndex(vocab_path, voc_indices_path);

    // start the viewer
    ulocalization.View();
}
