#include "test_utils.h"

// include opencv libraries, only used here in the example
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace colmap;

std::string path = "/home/viki/SFM/projects/database.db";
std::string map_path = "/home/viki/SFM/projects/2/";
std::string tree_path = "/home/viki/SFM/projects/vocabs/vocab_tree_flickr100K_words256K.bin";
std::string vocIndex_path = "/home/viki/SFM/projects/VocIndex.bin";
//std::string ply_path = "/home/viki/SFM/projects/3dmodels/bunny/reconstruction/bun_zipper_res2.ply";
std::string ply_path = "/home/viki/SFM/projects/3dmodels/dragon_recon/dragon_vrip_res4.ply";
std::string fbow_path = "/home/viki/SFM/projects/fbowtest.fbow";
std::string fused_path = "/home/viki/SFM/projects/2/fused_winter.ply";

//int new_height = 480*1.5;
int new_width = 3648;
int new_height = 2736;

void test1();

void test2();

void simpleviewer();

void FBOWtest();

int main(int argc, char** argv) {

  //FBOWtest();
  simpleviewer();

  return EXIT_SUCCESS;
}

void FBOWtest()
{
  Ulocal::Ulocalization ulocalization(path, map_path);

  //ulocalization.MakeFBOWvocabulary(fbow_path);

  ulocalization.LoadFBOWvocabulary(fbow_path);

  ulocalization.AddImagesToVoc();

  std::string pathimg1 = "/home/viki/SFM/projects/imgs/IMG_20190808_162900.jpg";
  FeatureKeypoints* keypoints1 = new FeatureKeypoints;
  FeatureDescriptors* descriptors1 = new FeatureDescriptors;
  Ulocal::SIFTextractionTest(pathimg1,keypoints1,descriptors1, new_width,new_height);

  ulocalization.MatchImage(*descriptors1);

}

void simpleviewer()
{

  Ulocal::Ulocalization ulocalization(path, map_path);

  ulocalization.LoadPLY(ply_path);
 
  ulocalization.LoadFusedPointcloud(fused_path);

  ulocalization.View();

}

void test1()
{
  //int new_width = 640*1.5;

  //std::cout << "Hello test start." << std::endl;

  std::string pathimg1 = "/home/viki/SFM/projects/IMG_20190627_151612.jpg";
  FeatureKeypoints* keypoints1 = new FeatureKeypoints;
  FeatureDescriptors* descriptors1 = new FeatureDescriptors;
  Ulocal::SIFTextractionTest(pathimg1,keypoints1,descriptors1, new_width,new_height);

  image_t refIdx = 277;
  Ulocal::Ulocalization ulocalization(path, map_path);
  FeatureMatches matches = ulocalization.MatchWithImage(refIdx, *descriptors1);

  ulocalization.PoseEstimation(*keypoints1, refIdx, matches);

  ulocalization.View();
}

void test2()
{

  std::string pathimg1 = "/home/viki/SFM/projects/imgs/IMG_20190808_162812.jpg";
  FeatureKeypoints* keypoints1 = new FeatureKeypoints;
  FeatureDescriptors* descriptors1 = new FeatureDescriptors;
  Ulocal::SIFTextractionTest(pathimg1,keypoints1,descriptors1, new_width,new_height);

  Ulocal::Ulocalization ulocalization(path, map_path);

  //ulocalization.MakeVocTreeIndex(tree_path, vocIndex_path);
   
  image_t refIdx = ulocalization.MatchVocTree(vocIndex_path, * keypoints1, * descriptors1);

  FeatureMatches matches = ulocalization.MatchWithImage(refIdx, *descriptors1);

  ulocalization.PoseEstimation(*keypoints1, refIdx, matches);

  ulocalization.View();

}
