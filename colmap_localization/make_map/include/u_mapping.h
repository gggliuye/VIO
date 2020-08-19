#ifndef U_MAPPING_H
#define U_MAPPING_H

#include <fstream>
#include <mutex>

#include "test_utils.h"
#include "ScaleEstimator.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "base/similarity_transform.h"
#include "controllers/automatic_reconstruction.h"
#include "controllers/bundle_adjustment.h"
#include "controllers/hierarchical_mapper.h"
#include "estimators/coordinate_frame.h"
#include "feature/extraction.h"
#include "feature/matching.h"
#include "feature/utils.h"
#include "mvs/meshing.h"
#include "mvs/patch_match.h"
#include "retrieval/visual_index.h"
#include "ui/main_window.h"
#include "util/opengl_utils.h"
#include "util/version.h"

#include "StateCallbacks.h"

// file system C++ need C++17
//#include <filesystem>
#include <dirent.h>

#include <iostream>
#include<opencv2/opencv.hpp>

#define CUDA_ENABLED

using namespace colmap;

namespace UMapping{

enum MAPPING_STATE
{
    MAKEING_IMAGE = 0,
    FEATURE_EXTRACTING = 1,
    FEATURE_MATCHING = 2,
    SPARSE_RECONSTRUCTION = 3,
    DENSE_RECONSTRUCTION = 4,
    OUTPUT_PLY_AND_VOC = 5,
    IDLE = 6,
    UNKOWN_ERROR = -1,
};

typedef void(*REGISTER_CALLBACK)(char *, int);

class MakeMapLY
{
public:
    MakeMapLY(){
        pStateManager = new StateManager();
    }
    ~MakeMapLY(){}

    void SetVocabularyPaths(std::string path_small, std::string path_medium, std::string path_large);

    void ColmapMapping(std::string work_space, std::string resource_path, std::string pre_fix);

    void ExtractImages(std::string work_space, std::string resource_path, std::string pre_fix);

    void ColmapMappingAdditionImages(std::string work_space, std::string resource_path, std::string pre_fix);

    void SetBuildParameters(bool build_dense, int build_quality){
        quality = build_quality;
        make_dense_output = build_dense;
    }

    int GetMappingState();

    // run into some error, set the current state to error
    void StateIsError();

    void BuildFinish();

    void SendFlag(int flag);

private:
    // scale caluclator
    bool b_calculate_scale = false;
    double colmap_scale = -1.0;

    int max_image_to_use = 200;
    double ar_core_focus = 496.6;
    std::string arcore_image_path;
    std::string arcore_trajectory_path;
    int PhaseArcoreTrajectroyPath(std::string work_space, std::string resource_path);

    double CalculateScale(std::string database_path, std::string map_path, std::string vocIndex_path);

private:
    // check if the file exist
    bool is_file_exist(const std::string fileName);

    void FindResoucesPaths(std::string &resource_path, 
               std::vector<std::string> &image_paths, std::vector<std::string> &video_paths);

    int CountImageInFolder(std::string folder_path);

    // check if feature is in the image range
    bool inBorder(const cv::Point2f &pt, int COL, int ROW);

    // reduce the length by delete some marked elements
    void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status);

    // use fundamental property to reduce bad tracked points
    void rejectWithFundamental(std::vector<cv::Point2f> &last_pts, std::vector<cv::Point2f> &new_pts);

    // calculate parallax between two frames
    Eigen::Vector2d calculateParallax(std::vector<cv::Point2f> &last_pts, std::vector<cv::Point2f> &new_pts);

    // divide a video into images, use feature parallax as threshold
    // and save the images into the work space
    int MakeVideosIntoImages(const std::string work_space, const std::string projectPath, 
                              int idx, double feature_parallax_threshold_);

    // collect all the images from videos and also images
    int CollectAllTheImages(std::string &work_space, std::string &resource_path);

    std::string ChooseTheSuitableVocabulary(int num_images);
  
    void RunReconstructionWithoutDense(std::string projectPath, std::string vocPath);

    void MakeIndex(std::string database_path, std::string map_path, std::string work_space,
                      std::string voc_path, std::string vocIndex_path, std::string pre_fix);

    bool b_save_resized_image = false;
    float resize_ratio = 1.0;

public:
    void SetToSaveResizedImage(float resize_ratio_){
        b_save_resized_image = true;
        resize_ratio = resize_ratio_;
        if(resize_ratio < 0){
            resize_ratio = 1.0;
        }
        std::cout << "\n==> Set to resize image with ratio " << resize_ratio << std::endl;
    }


private:

    void FindExistResoucesPaths(std::string &work_space, 
               std::vector<std::string> &image_folder_paths);
    int AddNewImages(std::string &work_space, std::string &resource_path);

public:
    std::mutex m_state;
    MAPPING_STATE mapping_state = IDLE;

private:
    bool b_show_image = false;

    bool voc_path_set = false;
    int num_small_threshold = 1000;
    std::string voc_path_small;
    int num_medium_threshold = 10000;
    std::string voc_path_medium;
    std::string voc_path_large;

// parameters
public:
    void SetImageExtractionParameters(double feature_parallax_threshold_, double laplacian_threshold_){
        laplacian_threshold = laplacian_threshold_;
        feature_parallax_threshold = feature_parallax_threshold_;
    }

private:
    double feature_parallax_threshold = 50;
    double laplacian_threshold = 20;
    bool use_gpu = true;
    bool single_camera_per_folder = true;

// costumer designed parameters
private:
    int quality = 1;
    bool make_dense_output = true;
    std::string pre_fix_key;

// register callback function
public:
    void register_callback(REGISTER_CALLBACK P){
        registered_callback = P;
        have_registered = true;
    }

private:
    bool have_registered = false;
    REGISTER_CALLBACK registered_callback = nullptr;

public:
    StateManager *pStateManager;

};


class UAutomaticReconstructionController : public Thread {
public:
    enum class DataType { INDIVIDUAL, VIDEO, INTERNET };
    enum class Quality { LOW=0, MEDIUM=1, HIGH=2, EXTREME=3 };
    enum class Mesher { POISSON, DELAUNAY };

    struct Options {
        // The path to the workspace folder in which all results are stored.
        std::string workspace_path;

        // The path to the image folder which are used as input.
        std::string image_path;

        // The path to the mask folder which are used as input.
        std::string mask_path;

        // The path to the vocabulary tree for feature matching.
        std::string vocab_tree_path;

        // The type of input data used to choose optimal mapper settings.
        DataType data_type = DataType::INDIVIDUAL;

        // Whether to perform low- or high-quality reconstruction.
        Quality quality = Quality::HIGH;

        bool single_camera_per_folder = false;

        // Whether to use shared intrinsics or not.
        bool single_camera = false;

        // Which camera model to use for images.
        std::string camera_model = "SIMPLE_RADIAL";

        // Whether to perform sparse mapping.
        bool sparse = true;

        bool multiple_models = false;

        // Whether to perform dense mapping.
        bool dense = false;

        // The meshing algorithm to be used.
        Mesher mesher = Mesher::POISSON;

        // The number of threads to use in all stages.
        int num_threads = -1;

        // Whether to use the GPU in feature extraction and matching.
        bool use_gpu = true;

        // Index of the GPU used for GPU stages. For multi-GPU computation,
        // you should separate multiple GPU indices by comma, e.g., "0,1,2,3".
        // By default, all GPUs will be used in all stages.
        std::string gpu_index = "-1";
    };

    UAutomaticReconstructionController(
           const Options& options, ReconstructionManager* reconstruction_manager);

    void Stop() override;

    void SetMakeMapLY(MakeMapLY* pMakeMapLY_){
        pMakeMapLY = pMakeMapLY_;
        pStateManager = pMakeMapLY_->pStateManager;
    }

private:
    MakeMapLY* pMakeMapLY;
    StateManager *pStateManager;

    void StateIsError(){
        pMakeMapLY->StateIsError();
    }

private:
    void Run() override;
    void RunFeatureExtraction();
    void RunFeatureMatching();
    void RunSparseMapper();
    void RunDenseMapper();

    const Options options_;
    OptionManager option_manager_;
    ReconstructionManager* reconstruction_manager_;

    Thread* active_thread_;
    std::unique_ptr<Thread> feature_extractor_;
    std::unique_ptr<Thread> exhaustive_matcher_;
    std::unique_ptr<Thread> sequential_matcher_;
    std::unique_ptr<Thread> vocab_tree_matcher_;

};
   

}  //namespace

#endif //U_MAPPING_H
