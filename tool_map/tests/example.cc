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
 
//bool remakeVocIndex = false;
//std::string vocIndex_path = "/data/Maps/kexuecheng/VocIndex.bin";
//std::string path = "/data/Maps/kexuecheng/database.db";
//std::string map_path = "/data/Maps/kexuecheng/sparse/";

//std::string vocIndex_path = "/home/utopa/Maps/winter_update/VocIndexTest.bin";
//std::string path = "/home/utopa/Maps/winter_update/database.db";
//std::string map_path = "/home/utopa/Maps/winter_update/sparse/";

void testFileImages(const std::string &database_path, const std::string &sparse_map_path, 
          const std::string &voc_indices_path, const std::string &test_images_path, 
          double focus_length, int width, int height);

void simpleviewer(const std::string &database_path, const std::string &sparse_map_path);

//void saveJson();

int main(int argc, char** argv) {
    if(argc != 8)
    {
        std::cerr << std::endl << "Usage: ./example database_path sparse_map_path voc_indices_path \n"
                     << "       test_images_path focus_length width height \n";
        return 1;
    }

    double focus_length = atoi(argv[5]);
    int width = atoi(argv[6]);
    int height = atoi(argv[7]);

    std::cout << "\n Test image from : " << argv[4] << std::endl;
    std::cout << "   image focus length : " << focus_length << ", size : " 
              << width << " X " << height << std::endl  << std::endl;

    testFileImages(argv[1], argv[2], argv[3], argv[4], focus_length, width, height);

    return EXIT_SUCCESS;
}

void simpleviewer(const std::string &database_path, const std::string &sparse_map_path)
{
    Ulocal::Ulocalization ulocalization(database_path, sparse_map_path);
    ulocalization.View();
}

void testFileImages(const std::string &database_path, const std::string &sparse_map_path, 
          const std::string &voc_indices_path, const std::string &test_images_path, 
          double focus_length, int width, int height)
{
    std::cout << "Test SIFT GPU extract features, then use Voc tree to find match image" << std::endl;
    std::cout << "Use Pnp and optimization method to esimate camera pose." << std::endl;
    std::cout << "    --  Test Start" << std::endl << std::endl;


    // load the prebuilt colmap map file
    Ulocal::Ulocalization ulocalization(database_path, sparse_map_path);
    //ulocalization.PrintCameraInfo();

    //ulocalization.SavePLYCLoud("/data/Maps/kexuecheng.ply");

    // match using voc tree algorithm to find condidate image frame
    ulocalization.LoadVocTree(voc_indices_path);

    // create sift matcher gpu
    ulocalization.CreateGPUMatch();

/*
    // mata20
    std::string imageFile = "/home/utopa/Maps/testimages/";
    double focus_length = 3061.03;
    int new_width = 3968;
    int new_height = 2976;    
    // p20 pro
    std::string imageFile = "/home/utopa/Maps/winter_update/testimages/";
    double focus_length = 2814.17;
    int new_width = 3648;
    int new_height = 2736;   
*/
    int new_width = width;
    int new_height = height;   

    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (test_images_path.c_str())) == NULL) {
        std::cout << "[ERROR] cannot open folder ! " << std::endl;
        return;
    }

    int numSuccess = 0;
    int totalImages = 0;
    int whatever = 0;
    while ((ent = readdir (dir)) != NULL) {
        std::cout << " [START] input new image : " << ent->d_name  << " " 
                  << ++whatever << std::endl << std::endl;
        std::string pathimg1 = test_images_path + ent->d_name;
 
        if(pathimg1.length() - test_images_path.length() < 4){
            continue;
        }

        TicToc tictoc;

        // SIFT GPU feature extraction
        FeatureKeypoints* keypoints1 = new FeatureKeypoints;
        FeatureDescriptors* descriptors1 = new FeatureDescriptors;
        bool succ = Ulocal::SIFTextractionTestGPU(pathimg1,keypoints1,descriptors1,
                                           new_width,new_height);
        if(!succ){
            break;
        }

        totalImages++;
        Ulocal::Retrieval retrieval = ulocalization.MatchVocTreeReturnAll(*keypoints1, *descriptors1);

        int tmp = 1;
        int maxTrival = 10;
        for(auto image_score : retrieval.image_scores){
            //std::cout << "Test the " << tmp << "th candidate: " << std::endl;
            tmp++;
            FeatureMatches matches = ulocalization.MatchWithImageGPU(image_score.image_id, *descriptors1);
            //FeatureMatches matches = ulocalization.MatchWithImage(image_score.image_id, *descriptors1);
            if(ulocalization.PoseEstimation(*keypoints1, image_score.image_id, matches, 
                        focus_length, new_width, new_height))
            {
                numSuccess++;
                break;
            }
            if(tmp > maxTrival){
                std::cout << " Have made too much tests. " << std::endl;
                std::cout << " [FAILED] Localization failed. " << std::endl;
                break;
            }
        }

        std::cout  << " [FINISH]  Total time used : " 
                   << tictoc.Now() << std::endl << std::endl;
    }
    
    std::cout << " Total image number : " << totalImages  << ", success number : " << numSuccess << std::endl;

    // load the ply model virtual object
    //ulocalization.LoadPLY(ply_path);
 
    // load the fused semi-dense point cloud
    //ulocalization.LoadFusedPointcloud(downsampled_cloud);

    // start the viewer
    ulocalization.View();
}


/*
void saveJson()
{
    Ulocal::Ulocalization ulocalization(path, map_path);
    ulocalization.LoadPlyAndSaveJson(downsampled_cloud, downsampled_cloud_json_save);
}
*/
/*
void FBOWtest()
{
  Ulocal::Ulocalization ulocalization(path, map_path);

  //ulocalization.MakeFBOWvocabulary(fbow_path);

  ulocalization.LoadFBOWvocabulary(fbow_path);

  ulocalization.AddImagesToVoc();

  std::string pathimg1 = "/home/viki/SFM/projects/imgs/IMG_20190808_162900.jpg";
  int new_width = 3968;
  int new_height = 2976;

  FeatureKeypoints* keypoints1 = new FeatureKeypoints;
  FeatureDescriptors* descriptors1 = new FeatureDescriptors;
  Ulocal::SIFTextractionTest(pathimg1,keypoints1,descriptors1, new_width,new_height);

  ulocalization.MatchImage(*descriptors1);

}
*/
