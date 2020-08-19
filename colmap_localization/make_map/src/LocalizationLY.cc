#include "LocalizationLY.h"

namespace Ulocal{

LocalizationLY::LocalizationLY(const std::string& database_path, const std::string& recs_path, const std::string& index_path,
                         bool bViewer_, bool bSaveResult_)
{
    bViewer = bViewer_;
    bSaveResult = bSaveResult_;

    database = new Database(database_path);
    reconstruction = new Reconstruction;
    reconstruction->ReadBinary(recs_path);

    // read the camera
    // camera = reconstruction->Camera(1);
    numCamera = reconstruction->NumCameras();

    // receive all keyframes
    framesIds.clear();
    framesIds = reconstruction->RegImageIds();

    if(bViewer){
        pViewerLY = new ViewerLY();

        // receive all map points
        std::unordered_set<colmap::point3D_t> mapPointsIds = reconstruction->Point3DIds();
        std::vector<colmap::Point3D> mapPoints;
        mapPoints.clear();
        mapPoints.reserve(reconstruction->NumPoints3D());
        for (const auto idx : mapPointsIds )
        {
            colmap::Point3D pt = reconstruction->Point3D(idx);
            mapPoints.push_back(pt);
        }
        
        pViewerLY->SetMapPoints(mapPoints);

        for (const auto idx : framesIds ){
            // Compose the inverse projection matrix from image to world space
            Eigen::Matrix3x4d pose = reconstruction->Image(idx).InverseProjectionMatrix();
            pViewerLY->AddKeyFrame(pose);
        }
    }

    LoadVocTree(index_path);

    std::cout << " [MATCH GPU] create sift matcher gpu. " << std::endl;

    if(!siftMatcherGPUcreated){ 
        match_options.num_threads = 4;
        match_options.max_num_matches = 13000;

        sift_match_gpu = new SiftMatchGPU;
        CreateSiftGPUMatcher(match_options, sift_match_gpu);
        siftMatcherGPUcreated = true;
    }
    
    numImage = reconstruction->NumImages();

    std::cout << std::endl << " [MAP INFO] sparse feature map  " << std::endl; 
    std::cout << "            camera number : "<< reconstruction->NumCameras() << std::endl;
    std::cout << "            point number : "<< reconstruction->NumPoints3D() << std::endl;
    std::cout << "            image number : "<< reconstruction->NumImages() << std::endl << std::endl;

    if(bSaveResult)
    {
        runtimeFilename = recs_path + "runtime_result.txt";
        std::cout << " [SAVE OUTPUT] save output to " << runtimeFilename << std::endl;
        runtimefile.open(runtimeFilename.c_str());
        runtimefile << std::fixed;
    }
}

LocalizationLY::~LocalizationLY()
{
    if(database)
        database->Close();
}

void LocalizationLY::GetMapPoints(std::vector<Eigen::Vector4d> &vPoses, std::vector<Eigen::Vector3d> &vColors)
{
    // receive all map points
    std::unordered_set<colmap::point3D_t> mapPointsIds = reconstruction->Point3DIds();
    vPoses.clear();
    vColors.clear();
    vPoses.reserve(reconstruction->NumPoints3D());
    vColors.reserve(reconstruction->NumPoints3D());
    for(const auto idx : mapPointsIds ){
        colmap::Point3D pt = reconstruction->Point3D(idx);
        Eigen::Vector4d pt_eigen(pt.XYZ()(0), pt.XYZ()(1), pt.XYZ()(2), 1.0);
        Eigen::Vector3d pt_color(pt.Color(0), pt.Color(1), pt.Color(2));
        vPoses.push_back(pt_eigen);
        vColors.push_back(pt_color);
    }
}

void LocalizationLY::LoadVocTree(const std::string& index_path)
{
    std::cout << " [VOC TREE]  Read indexs : " << std::endl;
    visual_index.Read(index_path);
    std::cout << "             Calculating TF-IDF . " << std::endl << std::endl;
    visual_index.Prepare();
}

void LocalizationLY::View()
{
    if(bViewer && pViewerLY){
        pViewerLY->View();
    }
}

bool LocalizationLY::LocalizeImage(cv::Mat &image, double focus_length, 
                         Eigen::Vector4d &qvec, Eigen::Vector3d &tvec, bool bAsInitEstimate)
{
#if SAVE_TIME
    TicToc tictoc_whole;
    TicToc tictoc_sift;
#endif

    FeatureKeypoints* keypoints_t = new FeatureKeypoints;
    FeatureDescriptors* descriptors_t = new FeatureDescriptors;
    SIFTextractionGPU(image, keypoints_t, descriptors_t);

#if SAVE_TIME
    double sift_time = tictoc_sift.Now();
    TicToc tictoc_matchvoc;
#endif

    Retrieval retrieval;
    if(bAsInitEstimate){
        std::cout << " [INIT POSE] : " << qvec.transpose() << " " << tvec.transpose() << "\n";
        retrieval = MatchWithInitialPoseEstimation(qvec, tvec);
    } else {
        retrieval = MatchVocTreeReturnAll(*keypoints_t, *descriptors_t);
    }

#if SAVE_TIME
    double voc_time = tictoc_matchvoc.Now();
    TicToc tictoc_pose_esti;
#endif

    int new_width = image.cols;
    int new_height = image.rows;   

    bool ret = false;
    int tmp = 1;

    int maxTrival_;
    if(bAsInitEstimate){
        maxTrival_ = maxTrival_init_pose;
    } else {
        maxTrival_ = maxTrival;
    }

    //Eigen::Vector4d qvec;
    //Eigen::Vector3d tvec;
    for(auto image_score : retrieval.image_scores){
        tmp++;
        FeatureMatches matches = MatchWithImageGPU(image_score.image_id, *descriptors_t);

        if(PoseEstimation(*keypoints_t, image_score.image_id, matches, 
                    focus_length, new_width, new_height, qvec, tvec))
        {
            ret = true;
            break;
        }
        if(tmp > maxTrival_){
            std::cout << " Have made too much tests. " << std::endl;
            std::cout << " [FAILED] Localization failed. " << std::endl;
            ret = false;
            break;
        }
    }

    //std::cout << std::endl;

    if(bSaveResult && ret)
    {
        runtimefile << qvec(0) << " " << qvec(1) << " " << qvec(2) << " " << qvec(3) << " "
                    << tvec(0) << " " << tvec(1) << " " << tvec(2);
#if SAVE_TIME
        double pose_time = tictoc_pose_esti.Now();
        runtimefile << " " << sift_time << " " << voc_time << " " << pose_time << " " << tictoc_whole.Now();
#endif
        runtimefile << std::endl;
        
    }

    return ret;
}

int count = 0;
bool LocalizationLY::SIFTextractionGPU(cv::Mat &image,  FeatureKeypoints* keypoints,
                                       FeatureDescriptors* descriptors)
{
    cv::flip(image, image, 0);

    FIBITMAP* freeimagebitmap = FreeImage_ConvertFromRawBitsEx(false, image.data, FIT_BITMAP, 
                      image.cols, image.rows, image.step, image.elemSize()*8, 
                      FI_RGBA_RED_MASK, FI_RGBA_GREEN_MASK, FI_RGBA_BLUE_MASK);

    FIBITMAP* converted_bitmap = FreeImage_ConvertToGreyscale(freeimagebitmap);
    //freeimagebitmap = std::unique_ptr<FIBITMAP, decltype(&FreeImage_Unload)>(converted_bitmap, &FreeImage_Unload);

    Bitmap bitmap(converted_bitmap);

    //bitmap.Write("/data/testbit" + std::to_string(count++) + ".png");

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    SiftExtractionOptions options;
    options.num_threads = 4;
    options.use_gpu = true;

    std::cout << " init sift gpu extractor. \n";

    std::unique_ptr<SiftGPU> sift_gpu;
    sift_gpu.reset(new SiftGPU);
    if (!CreateSiftGPUExtractor(options, sift_gpu.get())) {
        std::cerr << "ERROR: SiftGPU not fully supported." << std::endl;
        return false;
    }
 
    std::cout << " start sift feature extraction. \n";

    bool tmp = ExtractSiftFeaturesGPU(
              options, bitmap, sift_gpu.get(),
              keypoints, descriptors);

    if(bVerbose){
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        std::cout << " [SIFT GPU] sift extraction : " << keypoints->size() << " points . Used time : " << ttrack << std::endl;
    }

    return tmp;
}

bool LocalizationLY::SIFTextractionCPU(cv::Mat &image, FeatureKeypoints* keypoints,
                FeatureDescriptors* descriptors)
{
    cv::flip(image, image, 0);

    FIBITMAP* freeimagebitmap = FreeImage_ConvertFromRawBitsEx(false, image.data, FIT_BITMAP, 
                      image.cols, image.rows, image.step, image.elemSize()*8, 
                      FI_RGBA_RED_MASK, FI_RGBA_GREEN_MASK, FI_RGBA_BLUE_MASK);

    FIBITMAP* converted_bitmap = FreeImage_ConvertToGreyscale(freeimagebitmap);
    //freeimagebitmap = std::unique_ptr<FIBITMAP, decltype(&FreeImage_Unload)>(converted_bitmap, &FreeImage_Unload);

    Bitmap bitmap(converted_bitmap);

    //bitmap.Write("/data/testbit" + std::to_string(count++) + ".png");

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    SiftExtractionOptions options;
    options.num_threads = 4;
    options.use_gpu = false;

    bool tmp = ExtractSiftFeaturesCPU(options, bitmap, keypoints, descriptors);
  
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    std::cout << " [SIFT CPU] sift extraction : " << keypoints->size() << " points . Used time : " << ttrack << std::endl;
    return tmp;
}

Retrieval LocalizationLY::MatchVocTreeReturnAll(FeatureKeypoints &keypoints, FeatureDescriptors &descriptors)
{
    TicToc tictoc;

    VisualIndex_LY<>::QueryOptions query_options;
    query_options.max_num_images = numImage;
    //query_options.max_num_images = 10;
    query_options.num_images_after_verification = 0;

    Retrieval retrieval;
    retrieval.image_id = 0;
    visual_index.Query(query_options, keypoints, descriptors,
                        &retrieval.image_scores);

    if(bVerbose){
        std::cout << " [VOC TREE] retrieval all image candidates used time : " << tictoc.Now() << std::endl;
    }
    return retrieval;
}

double LocalizationLY::CompareQuaternions(Eigen::Vector4d &qvec_1, Eigen::Vector4d &qvec_2)
{
    Eigen::Vector3d unit_x(1.0,0.0,0.0);
    Eigen::Quaterniond q1(qvec_1(0), qvec_1(1), qvec_1(2), qvec_1(3)); 
    Eigen::Quaterniond q2(qvec_2(0), qvec_2(1), qvec_2(2), qvec_2(3)); 
    Eigen::Quaterniond delta_q = q1 * q2.inverse(); 
    Eigen::Vector3d delta_euler = delta_q.matrix().eulerAngles(2,1,0);

    double delta = delta_euler.lpNorm<1>(); // norm() squaredNorm()
    return delta;
}

void LocalizationLY::GetKeyFrames(std::vector<Eigen::Vector3d> &vT, std::vector<Eigen::Vector4d> &vQ)
{
    vT.clear();
    vT.reserve(framesIds.size());
    vQ.clear();
    vQ.reserve(framesIds.size());
    for (const auto idx : framesIds ){
        Eigen::Vector3d tvec_kf = reconstruction->Image(idx).Tvec();
        Eigen::Vector4d qvec_kf = reconstruction->Image(idx).Qvec();
        vT.push_back(tvec_kf);
        vQ.push_back(qvec_kf);
    }
}

Retrieval LocalizationLY::MatchWithInitialPoseEstimation(Eigen::Vector4d &qvec, Eigen::Vector3d &tvec)
{
    TicToc tictoc;

    Retrieval retrieval;
    retrieval.image_id = 0;

    // receive all keyframes
    for (const auto idx : framesIds ){
        // pose which is defined as the transformation from world to image space.
        Eigen::Vector3d tvec_kf = reconstruction->Image(idx).Tvec();
        Eigen::Vector4d qvec_kf = reconstruction->Image(idx).Qvec();

        double delta_angle = CompareQuaternions(qvec, qvec_kf);
        if(delta_angle < Threshold_angle){
            Eigen::Vector3d delta_distance_vec = tvec_kf - tvec;
            double delta_distance = delta_distance_vec.lpNorm<1>(); // norm() squaredNorm()
            if(true || delta_distance < Threshold_distance){
                retrieval::ImageScore imagescore;
                imagescore.image_id = idx;
                imagescore.score = - delta_distance * (0.7 + sin(delta_angle * 3.14159 / 180));
                retrieval.image_scores.push_back(imagescore);
            }
        }
    }

    auto SortFunc = [](const retrieval::ImageScore& score1, const retrieval::ImageScore& score2) {
        return score1.score > score2.score;
    };

    std::sort(retrieval.image_scores.begin(), retrieval.image_scores.end(), SortFunc);

    if(bVerbose){
        std::cout << " [DISTANCE SEARCH] retrieval " << retrieval.image_scores.size() 
                  << " image near used time : " << tictoc.Now() << std::endl;
    }
    return retrieval;
}

FeatureMatches LocalizationLY::MatchWithImageGPU(image_t idx, FeatureDescriptors &descriptors)
{
    FeatureMatches matches;
  
    if(!siftMatcherGPUcreated){
        std::cout << " [ERROR] sift gpu matcher not created." << std::endl;
        return matches;
    }

    if(!database->ExistsImage(idx)){
        std::cout << " [ERROR] image not exist." << std::endl;
        return matches;
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    FeatureKeypoints keypoints2 = database->ReadKeypoints(idx);
    FeatureDescriptors descriptors2 = database->ReadDescriptors(idx);

    MatchSiftFeaturesGPU(match_options, &descriptors, &descriptors2, sift_match_gpu, &matches);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    
    if(bVerbose && false){
        std::cout << " [MatchWithImageGPU] two image have matches : " << matches.size() ;
        std::cout << ". Matching uses time : " << ttrack << std::endl;
    }

    return matches;
}

FeatureMatches LocalizationLY::MatchWithImageCPU(image_t idx, FeatureDescriptors &descriptors)
{
    FeatureMatches matches;
    if(!database->ExistsImage(idx)){
        std::cout << " [ERROR] image not exist." << std::endl;
        return matches;
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    FeatureKeypoints keypoints2 = database->ReadKeypoints(idx);
    FeatureDescriptors descriptors2 = database->ReadDescriptors(idx);

    //match_options.num_threads = 4;
    MatchSiftFeaturesCPU(match_options, descriptors, descriptors2, &matches);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    
    if(bVerbose && true){
        std::cout << " [MatchWithImageCPU] two image have matches : " << matches.size() ;
        std::cout << ". Matching uses time : " << ttrack << std::endl;
    }

    return matches;

}

bool LocalizationLY::PoseEstimation(FeatureKeypoints &keypoints, image_t refIdx, FeatureMatches matches, double focus_length, int width, int height, Eigen::Vector4d &qvec, Eigen::Vector3d &tvec)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // create camera
    Camera cameraT;
    cameraT.SetCameraId(numCamera+1);
    cameraT.InitializeWithName("PINHOLE", focus_length, width, height);

    //////////////////////////////////////////////////////////////////////////////
    // Search for 2D-3D correspondences
    //////////////////////////////////////////////////////////////////////////////

    // Feature match has two elements : point2D_idx1 and point2D_idx2
    Image& corr_image = reconstruction->Image(refIdx);
    if (!corr_image.IsRegistered()) {
        return false;
    }

    std::vector<Eigen::Vector2d> tri_points2D;
    std::vector<Eigen::Vector3d> tri_points3D;
    for(size_t i = 0; i < matches.size(); i ++)
    {
        const Point2D& corr_point2D = corr_image.Point2D(matches[i].point2D_idx2);
        if (!corr_point2D.HasPoint3D()) {
            continue;
        }
        const Point3D& point3D = reconstruction->Point3D(corr_point2D.Point3DId());
        FeatureKeypoint featurept = keypoints.at(matches[i].point2D_idx1);
        Eigen::Vector2d point2D(featurept.x, featurept.y);
        tri_points2D.push_back(point2D);
        tri_points3D.push_back(point3D.XYZ());

    }

    //////////////////////////////////////////////////////////////////////////////
    // 2D-3D estimation
    //////////////////////////////////////////////////////////////////////////////
      
    AbsolutePoseEstimationOptions abs_pose_options;
    abs_pose_options.estimate_focal_length = false;
    // Use high confidence to avoid preemptive termination of P3P RANSAC
    // - too early termination may lead to bad registration.
    abs_pose_options.ransac_options.min_num_trials = 30;
    abs_pose_options.ransac_options.confidence = 0.9999;
    abs_pose_options.ransac_options.max_error = 3.0;

    AbsolutePoseRefinementOptions abs_pose_refinement_options;
    abs_pose_refinement_options.refine_focal_length = false;
    abs_pose_refinement_options.refine_extra_params = false;
    abs_pose_refinement_options.print_summary = false;

    size_t num_inliers;
    std::vector<char> inlier_mask;

    //Eigen::Vector4d qvec; 
    //Eigen::Vector3d tvec;

    if (!EstimateAbsolutePose(abs_pose_options, tri_points2D, tri_points3D,
                            &qvec, &tvec, &cameraT, &num_inliers,
                            &inlier_mask)){
        return false;
    }

    if(num_inliers < 20){
        return false;
    }

    if (!RefineAbsolutePose(abs_pose_refinement_options, inlier_mask,
                          tri_points2D, tri_points3D, &qvec,
                          &tvec, &cameraT)) {
        return false;
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    
    if(bVerbose){
        std::cout << "    Result pose : " << std::endl;
        std::cout << "          q : " << qvec.transpose() << std::endl;
        std::cout << "          t : " << tvec.transpose() << std::endl;
        std::cout << "    with " << num_inliers << " inliers." << std::endl;
        std::cout << "    Time used :  " << ttrack << " second" << std::endl;
    }

    if(bViewer){
        pViewerLY->SetCurrentPose(qvec, tvec);
    }

    return true;

}

}  // namespace
