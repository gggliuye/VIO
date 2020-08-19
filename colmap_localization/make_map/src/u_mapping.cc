#include "u_mapping.h"

#include "controllers/automatic_reconstruction.h"

#include "base/undistortion.h"
#include "controllers/incremental_mapper.h"
#include "feature/extraction.h"
#include "feature/matching.h"
#include "mvs/fusion.h"
#include "mvs/meshing.h"
#include "mvs/patch_match.h"
#include "util/misc.h"
#include "util/option_manager.h"

using namespace colmap;

namespace UMapping{

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

void MakeMapLY::SetVocabularyPaths(std::string path_small, 
                 std::string path_medium, std::string path_large)
{
    std::cout << path_small << "\n";
    std::cout << path_medium << "\n";
    std::cout << path_large << "\n";

    voc_path_small = path_small;
    voc_path_medium = path_medium;
    voc_path_large = path_large;
    voc_path_set = true;
}

bool MakeMapLY::is_file_exist(const std::string fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}

int MakeMapLY::GetMappingState()
{
    m_state.lock();
    int res = mapping_state;
    m_state.unlock();
    return res;    
}

void MakeMapLY::StateIsError()
{
    m_state.lock();
    mapping_state = UNKOWN_ERROR;
    m_state.unlock();
    SendFlag(UNKOWN_ERROR);
}

void MakeMapLY::SendFlag(int flag)
{
    //std::cout << " SendFlag step0 : " << flag << "\n";
    if(have_registered){
        char* key = new char[pre_fix_key.length() + 1];
        pre_fix_key.copy(key, std::string::npos);
        key[pre_fix_key.length()] = 0;
        //std::cout << " SendFlag step1: " << flag << "\n";
        registered_callback(key, flag);
    }
}

void MakeMapLY::BuildFinish()
{
    m_state.lock();
    mapping_state = IDLE;
    m_state.unlock();
    SendFlag(IDLE);
}

void MakeMapLY::ColmapMapping(std::string work_space, std::string resource_path, std::string pre_fix)
{
    std::cout << "\n    --  Map Building Process Start  -- " << std::endl 
              << " work space is : " << work_space << std::endl
              << " resource path is : " << resource_path << std::endl
              << std::endl;

    pre_fix_key = pre_fix;

    int num_images = CollectAllTheImages(work_space, resource_path);

    num_images += PhaseArcoreTrajectroyPath(work_space, resource_path);

    std::string voc_path = ChooseTheSuitableVocabulary(num_images);

    std::cout << " The voc file we choose is : " << voc_path << "\n";

    //std::string voc_path = voc_path_small;
    if(!is_file_exist(voc_path)){
        std::cout << " Could not find the voc file : " << voc_path << "\n";
        StateIsError();
        return;
    }
    RunReconstructionWithoutDense(work_space, voc_path);

    {
        std::string cmd_cp = "cp " + work_space + "/sparse/0/* " + work_space + "/sparse/";
        system(cmd_cp.data());
    }

    {
        //std::string cmd_cp = "rm -rf " + work_space + "/sparse/0";
        //system(cmd_cp.data());
    }

    m_state.lock();
    mapping_state = OUTPUT_PLY_AND_VOC;
    m_state.unlock();
    SendFlag(OUTPUT_PLY_AND_VOC);

    std::string vocIndex_path = work_space + "/VocIndex.bin";
    std::string database_path = work_space + "/database.db";
    std::string map_path = work_space + "/sparse/";

    std::cout << " -- Make Index for server localization -- " << std::endl << std::endl;
    {
        std::string cmd_cp = "cp " + work_space + "/dense/0/fused.ply " + work_space + "/" + pre_fix + "_dense.ply";
        system(cmd_cp.data());
    }
    MakeIndex(database_path, map_path, work_space, voc_path, vocIndex_path, pre_fix);

    // calculate scale
    double scale = CalculateScale(database_path, map_path, vocIndex_path);
    std::cout << " [SCALE ESTIMATION] scale result is : " << scale << "\n";

    m_state.lock();
    mapping_state = IDLE;
    m_state.unlock();
    SendFlag(IDLE);
}


void MakeMapLY::ExtractImages(std::string work_space, std::string resource_path, std::string pre_fix)
{
    std::cout << "\n    --  Extract Images Process Start  -- " << std::endl 
              << " work space is : " << work_space << std::endl
              << " resource path is : " << resource_path << std::endl
              << std::endl;

    std::cout << "\n - Image extraction parallax threshold : " << feature_parallax_threshold << std::endl;
    std::cout << " - Image extraction laplacian threshold : " << laplacian_threshold << std::endl << std::endl;

    pre_fix_key = pre_fix;

    int num_images = CollectAllTheImages(work_space, resource_path);

    num_images += PhaseArcoreTrajectroyPath(work_space, resource_path);

    m_state.lock();
    mapping_state = IDLE;
    m_state.unlock();
    SendFlag(IDLE);
}

void MakeMapLY::MakeIndex(std::string database_path, std::string map_path, std::string work_space,
                      std::string voc_path, std::string vocIndex_path, std::string pre_fix)
{
    Ulocal::Ulocalization ulocalization(database_path, map_path);
    //ulocalization.PrintCameraInfo();
    ulocalization.SavePLYCLoud(work_space + "/" + pre_fix + "_sparse.ply");
    ulocalization.MakeVocTreeIndex(voc_path, vocIndex_path);
}


void MakeMapLY::ColmapMappingAdditionImages(std::string work_space, std::string resource_path, std::string pre_fix)
{
    std::cout << "\n    --  Map Building Process Start  -- " << std::endl 
              << " work space is : " << work_space << std::endl
              << " resource path is : " << resource_path << std::endl
              << std::endl;

    pre_fix_key = pre_fix;

    int num_images = AddNewImages(work_space, resource_path);
    num_images += PhaseArcoreTrajectroyPath(work_space, resource_path);

    std::string voc_path = ChooseTheSuitableVocabulary(num_images);

    RunReconstructionWithoutDense(work_space, voc_path);

    {
        std::string cmd_cp = "cp " + work_space + "/sparse/0/* " + work_space + "/sparse/";
        system(cmd_cp.data());
    }


    m_state.lock();
    mapping_state = OUTPUT_PLY_AND_VOC;
    m_state.unlock();
    SendFlag(OUTPUT_PLY_AND_VOC);

    std::string vocIndex_path = work_space + "/VocIndex.bin";
    std::string database_path = work_space + "/database.db";
    std::string map_path = work_space + "/sparse/";

    //std::string voc_path = voc_path_small;
    std::cout << "\n -- Make Index for server localization -- " << std::endl << std::endl;
    {
        std::string cmd_cp = "cp " + work_space + "/dense/0/fused.ply " + work_space + "/" + pre_fix + "_dense.ply";
        system(cmd_cp.data());
    }
    MakeIndex(database_path, map_path, work_space, voc_path, vocIndex_path, pre_fix);

    m_state.lock();
    mapping_state = IDLE;
    m_state.unlock();
    SendFlag(IDLE);
    //BuildFinish();
}


int MakeMapLY::CollectAllTheImages(std::string &work_space, std::string &resource_path)
{
    std::vector<std::string> image_paths;
    std::vector<std::string> video_paths;
    FindResoucesPaths(resource_path, image_paths, video_paths);
    {
        std::string cmd = "mkdir " + work_space+ "/images";
        system(cmd.data());
    }
    // save all the video images
    size_t image_count = 0;
    for(size_t i = 0; i < video_paths.size(); i++){
        image_count += MakeVideosIntoImages(work_space, video_paths[i], 
                                        i, feature_parallax_threshold);
    }

    // save all the photo images
    std::string imageWritePath = work_space + "/images/";
    {
        //std::string cmd = "mkdir " + imageWritePath;
        //system(cmd.data());
    }
    for(size_t i = 0; i < image_paths.size(); i++){
        std::string cmd = "cp -rf " + image_paths[i] + " " + imageWritePath + "/";
        system(cmd.data());
        image_count += CountImageInFolder(image_paths[i]);
    }
    
    std::cout << " We have " << image_count << " images in total. \n";
    return image_count;
}

int MakeMapLY::AddNewImages(std::string &work_space, std::string &resource_path)
{  
    // count exist images
    int exist_image_count = 0;
    std::vector<std::string> image_folder_paths;
    FindExistResoucesPaths(work_space, image_folder_paths);
    int num_folder = image_folder_paths.size();
    for(int i = 0; i < num_folder; i++){
        exist_image_count += CountImageInFolder(image_folder_paths[i]);
    }
    std::cout << " There exist " << exist_image_count << " in the work space.\n";

    // add new images
    std::vector<std::string> image_paths;
    std::vector<std::string> video_paths;
    FindResoucesPaths(resource_path, image_paths, video_paths);

    // save all the video images
    size_t image_count = 0;
    for(size_t i = 0; i < video_paths.size(); i++){
        int idx = i + num_folder;
        image_count += MakeVideosIntoImages(work_space, video_paths[i], 
                                        idx, feature_parallax_threshold);
    }

    // save all the photo images
    std::string imageWritePath = work_space + "/images/";
    for(size_t i = 0; i < image_paths.size(); i++){
        std::string cmd = "cp -rf " + image_paths[i] + " " + imageWritePath + "/";
        system(cmd.data());
        image_count += CountImageInFolder(image_paths[i]);
    }
    
    std::cout << " Added " << image_count << " images. \n";

    image_count += exist_image_count;
    std::cout << " We have " << image_count << " images in total. \n";
    return image_count;
}

//bool have_scale = false;
//double colmap_scale = -1.0;
int MakeMapLY::PhaseArcoreTrajectroyPath(std::string work_space, std::string resource_path)
{
    std::string resource_path_t = resource_path + "/"; 
    std::string arcore_images_folder = resource_path_t + "trajdata/images";

    DIR *dir;
    if ((dir = opendir (arcore_images_folder.c_str())) == NULL) {
        std::cerr << "[NO SCALE DATA] cannot open arcore images folder ! " << std::endl;
        b_calculate_scale = false;
        return 0;
    }

    std::cout << "[ARCORE IMAGES] Find AR core images from " << arcore_images_folder << "\n";
    arcore_image_path = work_space + "/images/arcore/";
    arcore_trajectory_path = work_space + "/Trajectory.txt";

    if ((dir = opendir (arcore_image_path.c_str())) != NULL) {
        std::cerr << "[ALREADY HAVE SCALE DATA] " << std::endl;
        b_calculate_scale = true;
        //image_count = CountImageInFolder(imageWritePath);
        return 0;
    } 

    {
        std::string cmd = "mkdir " + arcore_image_path;
        system(cmd.data());
    }
    // read images
    int image_count = 0;
    {
        std::string cmd = "cp -rf " + arcore_images_folder + "/* " + arcore_image_path;
        system(cmd.data());
        image_count = CountImageInFolder(arcore_image_path);
        b_calculate_scale = true;
    }
    // copy the trajectory
    {
        std::string trajectory_file = resource_path_t + "trajdata/Trajectory.txt";
        std::string cmd = "cp " + trajectory_file + " " + arcore_trajectory_path;
        system(cmd.data());
    }

    return image_count;
}


int MakeMapLY::CountImageInFolder(std::string folder_path)
{
    int count_t = 0;
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (folder_path.c_str())) == NULL) {
        std::cerr << "[ERROR] cannot open images folder ! " << std::endl;
        return 0;
    }
    while ((ent = readdir (dir)) != NULL) {
        std::string pathimg = folder_path + ent->d_name;
        if(pathimg.length() - folder_path.length() < 4){
            continue;
        }
        count_t++;
    }

    std::cout << " Folder " << folder_path << " has " << count_t << " images.\n";

    return count_t;
}

void MakeMapLY::FindResoucesPaths(std::string &resource_path, 
               std::vector<std::string> &image_paths, std::vector<std::string> &video_paths)
{
    m_state.lock();
    mapping_state = MAKEING_IMAGE;
    m_state.unlock();
    SendFlag(MAKEING_IMAGE);

    std::cout << " [MAKEING_IMAGE] Find video and images from " << resource_path << "\n";
    std::string resource_path_t = resource_path + "/"; 
    std::string image_folder = resource_path_t + "images/";
    std::string video_folder = resource_path_t + "videos/";
    // read images
    {
        image_paths.clear();
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir (image_folder.c_str())) == NULL) {
            std::cerr << "[ERROR] cannot open images folder ! " << std::endl;
            return;
        }
        while ((ent = readdir (dir)) != NULL) {
            std::string pathimg = image_folder + ent->d_name;
            if(pathimg.length() - image_folder.length() < 4){
                continue;
            }
            image_paths.push_back(pathimg);
        }
    }
    // read videos
    {
        video_paths.clear();
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir (video_folder.c_str())) == NULL) {
            std::cerr << "[ERROR] cannot open videos folder ! " << std::endl;
            return;
        }
        while ((ent = readdir (dir)) != NULL) {
            std::string pathvideo = video_folder + ent->d_name;
            if(pathvideo.length() - video_folder.length() < 4){
                continue;
            }
            video_paths.push_back(pathvideo);
        }
    }

    std::cout << " Have Read " << image_paths.size() << " image folders, and " 
              << video_paths.size() << " videos. \n\n";
}


void MakeMapLY::FindExistResoucesPaths(std::string &work_space, 
               std::vector<std::string> &image_folder_paths)
{
    std::string resource_path_t = work_space + "/"; 
    std::string image_folder = resource_path_t + "images/";

    // read images
    {
        image_folder_paths.clear();
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir (image_folder.c_str())) == NULL) {
            std::cerr << "[ERROR] cannot open images folder ! " << std::endl;
            return;
        }
        while ((ent = readdir (dir)) != NULL) {
            std::string pathimg = image_folder + ent->d_name;
            if(pathimg.length() - image_folder.length() < 4){
                continue;
            }
            image_folder_paths.push_back(pathimg);
        }
    }

    std::cout << " Had exist " << image_folder_paths.size() << " image folders. \n\n";
}

int MakeMapLY::MakeVideosIntoImages(const std::string work_space, 
            const std::string video_path, int idx, double feature_parallax_threshold_)
{
    //std::cout << " Find video file : " << video_path << "\n";
    cv::VideoCapture capture;
    cv::Mat frame;
    frame = capture.open(video_path);
    if(!capture.isOpened()){
        std::cout << " [ERROR] fail to open video " << video_path << "\n";
        return 0;
    }

    std::cout << " [START] seperating video : " << video_path << std::endl;

    std::string imageWritePath = work_space + "/images/video_" + std::to_string(idx);
    std::string cmd = "mkdir " + imageWritePath;
    system(cmd.data());

    int imageCount = 0;
    std::vector<cv::Point2f> tracked_features;
    cv::Mat mask;
    cv::Mat last_gray;
    bool if_save = true;
    Eigen::Vector2d acc_parallax(0,0);

    while(capture.read(frame)) {

        cv::Mat frame_show, image_gray;
        cv::resize(frame, frame_show, cv::Size(frame.cols/2, frame.rows/2));
        cv::cvtColor(frame_show, image_gray, CV_RGB2GRAY);

        int cols = frame_show.cols;
        int rows = frame_show.rows;
        int MIN_DIST = cols / 30;
        int MAX_CNT = cols * 1.5;
        double feature_parallax_threshold_t = feature_parallax_threshold_ * cols / 640;
        std::vector<cv::Point2f> tracking_pts;
        //double parallax = 0;
        if(tracked_features.size() > 0){
            // track last feature
            std::vector<uchar> status;
            std::vector<float> err;
            cv::calcOpticalFlowPyrLK(last_gray, image_gray, tracked_features, tracking_pts, 
                                     status, err, cv::Size(21, 21), 3);
            // if the points is outside the border, set the point flag to false
            for (int i = 0; i < int(tracking_pts.size()); i++)
                if (status[i] && !inBorder(tracking_pts[i], cols, rows))
                    status[i] = 0;
            reduceVector(tracking_pts, status);
            reduceVector(tracked_features, status);

            rejectWithFundamental(tracked_features, tracking_pts);

            Eigen::Vector2d parallax = calculateParallax(tracked_features, tracking_pts);
            double acc_parallax_score = std::abs(acc_parallax(0)) + std::abs(acc_parallax(1));
            if(acc_parallax_score < feature_parallax_threshold_t){
                acc_parallax += parallax;
                if_save = false;
            } else {
                cv::Mat imgLaplacian, image_gray_resized;
                resize(image_gray, image_gray_resized, cv::Size(640, 480));
                cv::Laplacian(image_gray_resized, imgLaplacian, CV_8UC1);
                cv::Scalar mean, stddev;
 
                cv::meanStdDev(imgLaplacian, mean, stddev);
                uchar stddev_pxl = stddev.val[0]; 
                float score_laplacian = (float)stddev_pxl;
                //std::cout << " " << score_laplacian << std::endl;
                //auto score_laplacian = cv::Laplacian(image_gray, CV_64F).var();
 
                // if lot too much connection, loose the Laplacian threshold
                float laplacian_threshold_t = laplacian_threshold;
                if(acc_parallax_score > feature_parallax_threshold_t * 2){
                    laplacian_threshold_t = laplacian_threshold / 2;
                }

                if(score_laplacian < laplacian_threshold_t){
                    // image is blurred
                    acc_parallax += parallax;
                    if_save = false;
                } else {
                    acc_parallax << 0.0, 0.0;
                    if_save = true;
                }
            }
            //std::cout << " parallax : " << parallax << "\n";
        }

        mask = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(255));
        for (unsigned int i = 0; i < tracking_pts.size(); i++){
            cv::circle(mask, tracking_pts[i], MIN_DIST, 0, -1);
        }

        // detect new features to track
        std::vector<cv::Point2f> new_points;
        cv::goodFeaturesToTrack(image_gray, new_points, MAX_CNT - tracking_pts.size(), 0.01, MIN_DIST, mask);
        for (auto &p : new_points){
            tracking_pts.push_back(p);
        }
        //std::cout << " new points : " << new_points.size() << " " << MAX_CNT << "\n";
        if((int)new_points.size() > MAX_CNT / 10){
            if_save = true;
        }

        tracked_features = tracking_pts;
        last_gray = image_gray;

        if(b_show_image){
            for(auto &p : tracked_features){
                cv::circle(frame_show,p,3,cv::Scalar(0,255,0),-1);
            }
            std::stringstream s;
            //s << "parallax : "; s << (int)parallax; s << " , acc_parallax : "; s << (int)acc_parallax;
            cv::putText(frame_show,s.str(),cv::Point(10,rows-10),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),2,16);

            cv::imshow("video frame",frame_show);
            char key = static_cast<char>(cv::waitKey(10));
            if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
                break;
            }
        }

        if(if_save){
            if(b_save_resized_image){
                cv::resize(frame, frame, cv::Size(frame.cols*resize_ratio, frame.rows*resize_ratio));
            }

            cv::imwrite(imageWritePath + "/" + std::to_string(imageCount) + ".jpg", frame);
            imageCount++;
        }
    }
    capture.release();
    std::cout << "  [ EXTRACT " << imageCount << " IMAGES ] \n";
    return imageCount;
}

bool MakeMapLY::inBorder(const cv::Point2f &pt, int COL, int ROW)
{
    const int BORDER_SIZE = 5;
    return BORDER_SIZE <= pt.x && pt.x < COL - BORDER_SIZE && BORDER_SIZE <= pt.y && pt.y < ROW - BORDER_SIZE;
}

void MakeMapLY::reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status)
{
    int j = 0;
    for (unsigned int i = 0; i < v.size(); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void MakeMapLY::rejectWithFundamental(std::vector<cv::Point2f> &last_pts, std::vector<cv::Point2f> &new_pts)
{
    if (new_pts.size() >= 8){
        std::vector<uchar> status;
        cv::findFundamentalMat(last_pts, new_pts, cv::FM_RANSAC, 1.0, 0.99, status);
        reduceVector(new_pts, status);
        reduceVector(last_pts, status);
    }
}

Eigen::Vector2d MakeMapLY::calculateParallax(std::vector<cv::Point2f> &last_pts, std::vector<cv::Point2f> &new_pts)
{
    Eigen::Vector2d parallax(0,0);
    //double parallax = 0;
    for(size_t i = 0 ; i < last_pts.size(); i ++){
        cv::Point2f pt_1 = last_pts[i];
        cv::Point2f pt_2 = new_pts[i];
        float dx = pt_1.x - pt_2.x;
        float dy = pt_1.y - pt_2.y;
        parallax(0) += dx;
        parallax(1) += dy;
        //parallax += sqrt(dx * dx + dy * dy);
    }
    parallax = parallax / last_pts.size();
    return parallax;
}

std::string MakeMapLY::ChooseTheSuitableVocabulary(int num_images)
{
    if(num_images < num_small_threshold)
        return voc_path_small;
    else if(num_images < num_medium_threshold)
        return voc_path_medium;
    else
        return voc_path_large;
}

double MakeMapLY::CalculateScale(std::string database_path, std::string map_path, std::string vocIndex_path)
{
    if(!b_calculate_scale){
        return -1;
    }

    Ulocal::LocalizationLY *pLocalizationLY;
    pLocalizationLY = new Ulocal::LocalizationLY(database_path, map_path, vocIndex_path, false, false);
    //std::string arcore_image_path;
    //std::string arcore_trajectory_path;
    return Estimator::ScaleEstimator(pLocalizationLY, arcore_trajectory_path, arcore_image_path, ar_core_focus, max_image_to_use);
}

void MakeMapLY::RunReconstructionWithoutDense(std::string projectPath, std::string vocPath)
{
    UAutomaticReconstructionController::Options reconstruction_options;

    // set workspace path
    reconstruction_options.workspace_path = projectPath;
    reconstruction_options.image_path = projectPath + "/images/";
    if(is_file_exist(vocPath)){
        reconstruction_options.vocab_tree_path = vocPath;
        std::cout << "\n Choose the Voc : " << vocPath << "\n";
    }

    reconstruction_options.use_gpu = use_gpu;

    // share parameters per folder
    reconstruction_options.single_camera_per_folder = single_camera_per_folder;

    // image match method
    reconstruction_options.data_type = 
        UAutomaticReconstructionController::DataType::INDIVIDUAL;

    //reconstruction_options.quality = 
    //    UAutomaticReconstructionController::Quality::LOW;

    switch(quality){
        case 0 :
            reconstruction_options.quality = 
                UAutomaticReconstructionController::Quality::LOW;
            break;
        case 1 :
            reconstruction_options.quality = 
                UAutomaticReconstructionController::Quality::MEDIUM;
            break;
        case 2 :
            reconstruction_options.quality = 
                UAutomaticReconstructionController::Quality::HIGH;
            break;
        case 3 :
            reconstruction_options.quality = 
                UAutomaticReconstructionController::Quality::EXTREME;
            break;
        default :
            reconstruction_options.quality = 
                UAutomaticReconstructionController::Quality::LOW;
            break;
    }
    //std::cout << " Reconstruction quality is " << reconstruction_options.quality << "\n";

    reconstruction_options.dense = make_dense_output;

    reconstruction_options.mesher = 
        UAutomaticReconstructionController::Mesher::DELAUNAY;

    ReconstructionManager reconstruction_manager;
    UAutomaticReconstructionController controller(reconstruction_options,
                                                 &reconstruction_manager);

    controller.SetMakeMapLY(this);

    controller.Start();
    controller.Wait();
}


UAutomaticReconstructionController::UAutomaticReconstructionController(
    const Options& options, ReconstructionManager* reconstruction_manager)
    : options_(options),
      reconstruction_manager_(reconstruction_manager),
      active_thread_(nullptr) {
  CHECK(ExistsDir(options_.workspace_path));
  CHECK(ExistsDir(options_.image_path));
  CHECK_NOTNULL(reconstruction_manager_);

  option_manager_.AddAllOptions();

  *option_manager_.image_path = options_.image_path;
  *option_manager_.database_path =
      JoinPaths(options_.workspace_path, "database.db");

  if (options_.data_type == DataType::VIDEO) {
    option_manager_.ModifyForVideoData();
  } else if (options_.data_type == DataType::INDIVIDUAL) {
    option_manager_.ModifyForIndividualData();
  } else if (options_.data_type == DataType::INTERNET) {
    option_manager_.ModifyForInternetData();
  } else {
    LOG(FATAL) << "Data type not supported";
  }

  CHECK(ExistsCameraModelWithName(options_.camera_model));

  if (options_.quality == Quality::LOW) {
    option_manager_.ModifyForLowQuality();
  } else if (options_.quality == Quality::MEDIUM) {
    option_manager_.ModifyForMediumQuality();
  } else if (options_.quality == Quality::HIGH) {
    option_manager_.ModifyForHighQuality();
  } else if (options_.quality == Quality::EXTREME) {
    option_manager_.ModifyForExtremeQuality();
  }

  option_manager_.sift_extraction->num_threads = options_.num_threads;
  option_manager_.sift_matching->num_threads = options_.num_threads;
  option_manager_.mapper->num_threads = options_.num_threads;
  option_manager_.poisson_meshing->num_threads = options_.num_threads;

  ImageReaderOptions reader_options = *option_manager_.image_reader;
  reader_options.database_path = *option_manager_.database_path;
  reader_options.image_path = *option_manager_.image_path;
  if (!options_.mask_path.empty()) {
    reader_options.mask_path = options_.mask_path;
    option_manager_.image_reader->mask_path = options_.mask_path;
  }
  reader_options.single_camera = options_.single_camera;
  reader_options.camera_model = options_.camera_model;
  reader_options.single_camera_per_folder = options_.single_camera_per_folder;

  option_manager_.sift_extraction->use_gpu = options_.use_gpu;
  option_manager_.sift_matching->use_gpu = options_.use_gpu;

  option_manager_.sift_extraction->gpu_index = options_.gpu_index;
  option_manager_.sift_matching->gpu_index = options_.gpu_index;
  option_manager_.patch_match_stereo->gpu_index = options_.gpu_index;

  // patch match options
  option_manager_.patch_match_stereo->num_samples = 15;
  option_manager_.patch_match_stereo->num_iterations = 5;
  option_manager_.patch_match_stereo->geom_consistency = true;


  feature_extractor_.reset(new SiftFeatureExtractor(
      reader_options, *option_manager_.sift_extraction));

  exhaustive_matcher_.reset(new ExhaustiveFeatureMatcher(
      *option_manager_.exhaustive_matching, *option_manager_.sift_matching,
      *option_manager_.database_path));

  if (!options_.vocab_tree_path.empty()) {
    option_manager_.sequential_matching->loop_detection = true;
    option_manager_.sequential_matching->vocab_tree_path =
        options_.vocab_tree_path;
  }

  sequential_matcher_.reset(new SequentialFeatureMatcher(
      *option_manager_.sequential_matching, *option_manager_.sift_matching,
      *option_manager_.database_path));

  if (!options_.vocab_tree_path.empty()) {
    option_manager_.vocab_tree_matching->vocab_tree_path =
        options_.vocab_tree_path;
    vocab_tree_matcher_.reset(new VocabTreeFeatureMatcher(
        *option_manager_.vocab_tree_matching, *option_manager_.sift_matching,
        *option_manager_.database_path));
  }

  option_manager_.mapper->multiple_models = options_.multiple_models;

}

void UAutomaticReconstructionController::Stop() {
  if (active_thread_ != nullptr) {
    active_thread_->Stop();
  }
  Thread::Stop();
}

void UAutomaticReconstructionController::Run() {
    if (IsStopped()) {
        StateIsError();
        return;
    }

    pMakeMapLY->m_state.lock();
    pMakeMapLY->mapping_state = FEATURE_EXTRACTING;
    pMakeMapLY->m_state.unlock();
    pMakeMapLY->SendFlag(FEATURE_EXTRACTING);

    RunFeatureExtraction();

    if (IsStopped()) {
        StateIsError();
        return;
    }

    pMakeMapLY->m_state.lock();
    pMakeMapLY->mapping_state = FEATURE_MATCHING;
    pMakeMapLY->m_state.unlock();
    pMakeMapLY->SendFlag(FEATURE_MATCHING);

    RunFeatureMatching();

    if (IsStopped()) {
        StateIsError();
        return;
    }

    pMakeMapLY->m_state.lock();
    pMakeMapLY->mapping_state = SPARSE_RECONSTRUCTION;
    pMakeMapLY->m_state.unlock();
    pMakeMapLY->SendFlag(SPARSE_RECONSTRUCTION);

    if (options_.sparse) {
        RunSparseMapper();
    }

    if (IsStopped()) {
        StateIsError();
        return;
    }

    pMakeMapLY->m_state.lock();
    pMakeMapLY->mapping_state = DENSE_RECONSTRUCTION;
    pMakeMapLY->m_state.unlock();
    pMakeMapLY->SendFlag(DENSE_RECONSTRUCTION);

    if (options_.dense) {
        RunDenseMapper();
    }
}

void UAutomaticReconstructionController::RunFeatureExtraction() {
    CHECK(feature_extractor_);
    active_thread_ = feature_extractor_.get();
    feature_extractor_->Start();
    feature_extractor_->Wait();
    feature_extractor_.reset();
    active_thread_ = nullptr;
}

void UAutomaticReconstructionController::RunFeatureMatching() {
    Thread* matcher = nullptr;
    if (options_.data_type == DataType::VIDEO) {
        matcher = sequential_matcher_.get();
    } else if (options_.data_type == DataType::INDIVIDUAL ||
               options_.data_type == DataType::INTERNET) {
        Database database(*option_manager_.database_path);
        const size_t num_images = database.NumImages();
        if (options_.vocab_tree_path.empty() || num_images < 200) {
            matcher = exhaustive_matcher_.get();
        } else {
            matcher = vocab_tree_matcher_.get();
        }
    }
  
    CHECK(matcher);
    active_thread_ = matcher;
    matcher->Start();
    matcher->Wait();
    exhaustive_matcher_.reset();
    sequential_matcher_.reset();
    vocab_tree_matcher_.reset();
    active_thread_ = nullptr;
}

void UAutomaticReconstructionController::RunSparseMapper() {
  const auto sparse_path = JoinPaths(options_.workspace_path, "sparse");
  if (ExistsDir(sparse_path)) {
    auto dir_list = GetDirList(sparse_path);
    std::sort(dir_list.begin(), dir_list.end());
    if (dir_list.size() > 0) {
      std::cout << std::endl
                << "WARNING: Skipping sparse reconstruction because it is "
                   "already computed"
                << std::endl;
      for (const auto& dir : dir_list) {
        reconstruction_manager_->Read(dir);
      }
      return;
    }
  }

  // callback functions
  bool b_init_pair_reg = false;
  std::function<void()> Init_image_ref_callback = [&b_init_pair_reg]() {
      b_init_pair_reg = true;
      std::cout << " [INITIAL_IMAGE_PAIR_REG_CALLBACK] " << b_init_pair_reg << "\n";
  };

  int num_registered_image = 3;
  std::function<void()> Next_image_reg_callback = [&num_registered_image]() {
      num_registered_image += 1;
      std::cout << " [NEXT_IMAGE_REG_CALLBACK] processed image count : " << num_registered_image << "\n";
  };

  bool b_last_image = false;
  std::function<void()> Last_image_ref_callback = [&b_last_image]() {
      b_last_image = true;
      std::cout << " [LAST_IMAGE_REG_CALLBACK] " << b_last_image << "\n";
  };

  IncrementalMapperController mapper(
      option_manager_.mapper.get(), *option_manager_.image_path,
      *option_manager_.database_path, reconstruction_manager_);
  mapper.AddCallback(IncrementalMapperController::INITIAL_IMAGE_PAIR_REG_CALLBACK, 
                      Init_image_ref_callback);
  mapper.AddCallback(IncrementalMapperController::NEXT_IMAGE_REG_CALLBACK, 
                      Next_image_reg_callback);
  mapper.AddCallback(IncrementalMapperController::LAST_IMAGE_REG_CALLBACK, 
                      Last_image_ref_callback);
  
  active_thread_ = &mapper;
  mapper.Start();
  mapper.Wait();
  active_thread_ = nullptr;

  CreateDirIfNotExists(sparse_path);
  reconstruction_manager_->Write(sparse_path, &option_manager_);

  //reconstruction.ExportPLY(export_path);

}

void UAutomaticReconstructionController::RunDenseMapper() {
#ifndef CUDA_ENABLED
  std::cout
      << std::endl
      << "WARNING: Skipping dense reconstruction because CUDA is not available."
      << std::endl;
  return;
#endif  // CUDA_ENABLED

  CreateDirIfNotExists(JoinPaths(options_.workspace_path, "dense"));

  for (size_t i = 0; i < reconstruction_manager_->Size(); ++i) {
    if (IsStopped()) {
      return;
    }

    const std::string dense_path =
        JoinPaths(options_.workspace_path, "dense", std::to_string(i));
    const std::string fused_path = JoinPaths(dense_path, "fused.ply");

    std::string meshing_path;
    if (options_.mesher == Mesher::POISSON) {
      meshing_path = JoinPaths(dense_path, "meshed-poisson.ply");
    } else if (options_.mesher == Mesher::DELAUNAY) {
      meshing_path = JoinPaths(dense_path, "meshed-delaunay.ply");
    }

    if (ExistsFile(fused_path) && ExistsFile(meshing_path)) {
      continue;
    }

    // Image undistortion.

    if (!ExistsDir(dense_path)) {
      CreateDirIfNotExists(dense_path);

      UndistortCameraOptions undistortion_options;
      undistortion_options.max_image_size =
          option_manager_.patch_match_stereo->max_image_size;
      COLMAPUndistorter undistorter(undistortion_options,
                                    reconstruction_manager_->Get(i),
                                    *option_manager_.image_path, dense_path);
      active_thread_ = &undistorter;
      undistorter.Start();
      undistorter.Wait();
      active_thread_ = nullptr;
    }

    if (IsStopped()) {
      return;
    }

    // Patch match stereo.
    {
      mvs::PatchMatchController patch_match_controller(
          *option_manager_.patch_match_stereo, dense_path, "COLMAP", "");
      active_thread_ = &patch_match_controller;
      patch_match_controller.Start();
      patch_match_controller.Wait();
      active_thread_ = nullptr;
    }

    if (IsStopped()) {
      return;
    }

    // Stereo fusion.

    if (!ExistsFile(fused_path)) {
      auto fusion_options = *option_manager_.stereo_fusion;
      const int num_reg_images = reconstruction_manager_->Get(i).NumRegImages();
      fusion_options.min_num_pixels =
          std::min(num_reg_images + 1, fusion_options.min_num_pixels);
      mvs::StereoFusion fuser(
          fusion_options, dense_path, "COLMAP", "",
          options_.quality == Quality::HIGH ? "geometric" : "photometric");
      active_thread_ = &fuser;
      fuser.Start();
      fuser.Wait();
      active_thread_ = nullptr;

      std::cout << "Writing output: " << fused_path << std::endl;
      WriteBinaryPlyPoints(fused_path, fuser.GetFusedPoints());
      mvs::WritePointsVisibility(fused_path + ".vis",
                                 fuser.GetFusedPointsVisibility());
    }

    if (IsStopped()) {
      return;
    }

    // Surface meshing.

    if (!ExistsFile(meshing_path)) {
      if (options_.mesher == Mesher::POISSON) {
        mvs::PoissonMeshing(*option_manager_.poisson_meshing, fused_path,
                            meshing_path);
      } else if (options_.mesher == Mesher::DELAUNAY) {
#ifdef CGAL_ENABLED
        mvs::DenseDelaunayMeshing(*option_manager_.delaunay_meshing, dense_path,
                                  meshing_path);
#else   // CGAL_ENABLED
        std::cout << std::endl
                  << "WARNING: Skipping Delaunay meshing because CGAL is "
                     "not available."
                  << std::endl;
        return;
#endif  // CGAL_ENABLED
      }
    }
  }
}


}



