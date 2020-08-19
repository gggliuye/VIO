#include "test_utils.h"
//#include "json/json.h"
#include "SiftGPU/SiftGPU.h"

using namespace colmap;

namespace Ulocal{

Ulocalization::Ulocalization()
{
    database = new Database();
}

Ulocalization::Ulocalization(const std::string& database_path, const std::string& recs_path)
{
    database = new Database(database_path);
    reconstruction = new Reconstruction;
    reconstruction->ReadBinary(recs_path);

    // read the camera
    camera = reconstruction->Camera(1);

    // receive all map points
    std::unordered_set<colmap::point3D_t> mapPointsIds = reconstruction->Point3DIds();

    mapPoints.clear();
    mapPoints.reserve(reconstruction->NumPoints3D());

    for (const auto idx : mapPointsIds )
    {
      colmap::Point3D pt = reconstruction->Point3D(idx);
      mapPoints.push_back(pt);
    }

    // receive all keyframes
    framesIds.clear();
    framesIds = reconstruction->RegImageIds();

    Twcs.clear();
    Twcs.reserve(reconstruction->NumRegImages());

    for (const auto idx : framesIds )
    {
      Eigen::Matrix3x4d pose = reconstruction->Image(idx).InverseProjectionMatrix();
      pangolin::OpenGlMatrix Twc;
      GetOpenGLCameraMatrix(pose, Twc);
      Twcs.push_back(Twc);
    }

    numCamera = reconstruction->NumCameras();

    std::cout << std::endl << " [MAP INFO] sparse feature map  " << std::endl; 
    std::cout << "            camera number : "<< reconstruction->NumCameras() << std::endl;
    std::cout << "            point number : "<< reconstruction->NumPoints3D() << std::endl;
    std::cout << "            image number : "<< reconstruction->NumImages() << std::endl << std::endl;
}

Ulocalization::~Ulocalization()
{
    if(database)
        database->Close();
}

void Ulocalization::setImageFilePath(const std::string& set_path, const std::string& image_path)
{
    imageFilePath = set_path;
    currentImagePath = image_path;
}

void Ulocalization::PrintCameraInfo()
{
    // read the cameras
    for (int i = 0 ;  i<numCamera ; i ++){
        Camera cameraTmp = reconstruction->Camera(i+1);
        std::cout << " the " << i+1 << "th camera: " << cameraTmp.ModelName() << std::endl;
        std::cout << "    (fx, fx, cx, cy) : (" << cameraTmp.ParamsToString() << ")" << std::endl;
    }
    std::cout << std::endl;
}


FeatureMatches Ulocalization::MatchWithImage(image_t idx, FeatureDescriptors &descriptors)
{
    FeatureMatches matches;
    if(!database->ExistsImage(idx))
    {
        std::cout << " [ERROR] image not exist" << std::endl;
        return matches;
    }
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    FeatureKeypoints keypoints2 = database->ReadKeypoints(idx);
    FeatureDescriptors descriptors2 = database->ReadDescriptors(idx);

    std::cout << " [MATCH] there are " << keypoints2.size() << " points in image " << idx << std::endl;

    // match test 
    match_options.num_threads = 4;
    MatchSiftFeaturesCPU(match_options, descriptors, descriptors2, &matches);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    std::cout << " two image have matches : " << matches.size() ;
    std::cout << ". Matching uses time : " << ttrack << std::endl;

    return matches;
}

void Ulocalization::CreateGPUMatch()
{
    std::cout << " [MATCH GPU] create sift matcher gpu. " << std::endl << std::endl;

    if(!siftMatcherGPUcreated){ 
        match_options.num_threads = 4;
        match_options.max_num_matches = 13000;

        sift_match_gpu = new SiftMatchGPU;
        CreateSiftGPUMatcher(match_options, sift_match_gpu);
        siftMatcherGPUcreated = true;
    }
}

FeatureMatches Ulocalization::MatchWithImageGPU(image_t idx, FeatureDescriptors &descriptors)
{
    FeatureMatches matches;
  
    if(!siftMatcherGPUcreated){
        std::cout << " [ERROR] sift gpu matcher not created." << std::endl;
        return matches;
    }

    if(!database->ExistsImage(idx))
    {
        std::cout << " [ERROR] image not exist." << std::endl;
        return matches;
    }
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    FeatureKeypoints keypoints2 = database->ReadKeypoints(idx);
    FeatureDescriptors descriptors2 = database->ReadDescriptors(idx);

    //std::cout << " [MATCH GPU] there are " << keypoints2.size() << " points in image " << idx << std::endl;

    MatchSiftFeaturesGPU(match_options, &descriptors, &descriptors2, sift_match_gpu, &matches);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    
    if(false){
        std::cout << " two image have matches : " << matches.size() ;
        std::cout << ". Matching uses time : " << ttrack << std::endl;
    }

    return matches;
}

void Ulocalization::View()
{
    testpangolin();
}

void Ulocalization::LoadVocTree(const std::string& index_path)
{
    std::cout << " [VOC TREE]  Read indexs : " << std::endl;
    visual_index.Read(index_path);

    std::cout << "             Calculating TF-IDF . " << std::endl << std::endl;
    visual_index.Prepare();

}

void Ulocalization::MakeVocTreeIndex(const std::string& tree_path, const std::string& write_path)
{
    std::cout << "  Make vocabulary tree index. " <<std::endl;

    VisualIndex_LY<> visual_index;
    visual_index.Read(tree_path);

    VisualIndex_LY<>::IndexOptions index_options;
    index_options.num_threads = 4;
    index_options.num_checks = 4;
    index_options.num_neighbors = 1;

    int max_num_features = 3000;

    int count = 0;
    for (const auto idx : framesIds ){
        if(!database->ExistsImage(idx)){
            continue;
        }
        count++;
        std::cout << StringPrintf("Indexing image %d [ %d / %d ]", idx, count, framesIds.size()) << std::endl;
        auto keypoints = database->ReadKeypoints(idx);
        auto descriptors = database->ReadDescriptors(idx);
        ExtractTopScaleFeatures(&keypoints, &descriptors, max_num_features);

        //Eigen::Matrix3x4d pose = reconstruction->Image(idx).InverseProjectionMatrix();

        visual_index.Add(index_options, idx, keypoints, descriptors);
        //PrintElapsedTime(timer);
    }
    std::cout << "  Calculating TF-IDF . " << std::endl;
    visual_index.Prepare();
    std::cout << "  Saving to file " << write_path << std::endl;
    visual_index.Write(write_path);
}


void Ulocalization::MakeVocTreeIndexForFloor(const std::string& tree_path, const std::string& write_path, int floor)
{
    std::cout << "  Make vocabulary tree index for the " << floor << "th floor. " <<std::endl;

    VisualIndex_LY<> visual_index;
    visual_index.Read(tree_path);

    VisualIndex_LY<>::IndexOptions index_options;
    index_options.num_threads = 4;
    index_options.num_checks = 4;

    int max_num_features = 3000;

    int count = 0;
    for (const auto idx : framesIds ){
        if(!database->ExistsImage(idx)){
            continue;
        }

        std::string imagePath = reconstruction->Image(idx).Name();
        std::string folderName(imagePath, 0, 5);

        if(!(folderName.compare("first") == 0))
            continue;

        count++;
        std::cout << "Image path : " << imagePath << std::endl;

        std::cout << StringPrintf(" - Indexing image [ %d / %d ]", idx + 1, framesIds.size()) << std::endl;
  
        auto keypoints = database->ReadKeypoints(idx);
        auto descriptors = database->ReadDescriptors(idx);
      
        ExtractTopScaleFeatures(&keypoints, &descriptors, max_num_features);

        visual_index.Add(index_options, idx, keypoints, descriptors);
    }

    std::cout << "  Have index " << count << " images. " << std::endl;
    std::cout << "  Calculating TF-IDF . " << std::endl;
    visual_index.Prepare();

    std::cout << "  Saving to file " << write_path << std::endl;
    visual_index.Write(write_path);
}


/*
* Voc tree will offer a lot of candidates
* However not all the candidates are good, or should I say, most of them are bad
* As a result still a lot of time taken to calulate match points between candidates
* input image. In order to obtain a satsifying result.
*/
image_t Ulocalization::MatchVocTree(FeatureKeypoints &keypoints, FeatureDescriptors &descriptors)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    VisualIndex_LY<>::QueryOptions query_options;
    query_options.max_num_images = Twcs.size();
    // if num_images_after_verification set to non positive, 
    // image retrival will out put all the candidates detected
    query_options.num_images_after_verification = 1;

    std::cout << " [VOC TREE] retrieval the best image candidate. " << std::endl;
    Retrieval retrieval;
    retrieval.image_id = 0;
    visual_index.Query(query_options, keypoints, descriptors,
                        &retrieval.image_scores);
    int tmp = 0;
    int max_show = 100;
    for(auto image_score : retrieval.image_scores){
        std::cout << "  - matched id : " << image_score.image_id << ", score : " << image_score.score;
        std::cout << ", file path: "<< reconstruction->Image(image_score.image_id).Name() << std::endl;
        tmp++;
        if(tmp > max_show){
            break;
        }
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    std::cout << "    Voc tree match used time : " << ttrack << std::endl << std::endl;

    return retrieval.image_scores[0].image_id;
}

Retrieval Ulocalization::MatchVocTreeReturnAll(FeatureKeypoints &keypoints, FeatureDescriptors &descriptors)
{
    TicToc tictoc;

    VisualIndex_LY<>::QueryOptions query_options;
    query_options.max_num_images = Twcs.size();
    //query_options.max_num_images = 10;
    query_options.num_images_after_verification = 0;

    std::cout << " [VOC TREE] retrieval all image candidates. " << std::endl;
    Retrieval retrieval;
    retrieval.image_id = 0;
    visual_index.Query(query_options, keypoints, descriptors,
                        &retrieval.image_scores);

    std::cout << " Voc tree match used time : " << tictoc.Now() << std::endl;

    return retrieval;
}


bool Ulocalization::PoseEstimation(FeatureKeypoints &keypoints, image_t refIdx, FeatureMatches matches, double focus_length, int width, int height)
{
    bool verboseLocal = false;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    if(verboseLocal){
        std::cout << " [POSE ESTIMATE] Pose Estimation start." << std::endl;
    }

    // create camera
    Camera cameraT;
    cameraT.SetCameraId(numCamera+1);
    cameraT.InitializeWithName("PINHOLE", focus_length, width, height);

    if(verboseLocal){
        std::cout << "   (fx, fx, cx, cy) : (" << cameraT.ParamsToString() << ")" << std::endl;
    }

    //////////////////////////////////////////////////////////////////////////////
    // Search for 2D-3D correspondences
    //////////////////////////////////////////////////////////////////////////////

    // Feature match has two elements : point2D_idx1 and point2D_idx2
    Image& corr_image = reconstruction->Image(refIdx);

    if(verboseLocal){
        std::cout << "   reference image : " << corr_image.Name() << std::endl;
    }

    if (!corr_image.IsRegistered()) {
        if(verboseLocal){
            std::cout << " [FAIL] corresponding image note registered. " << std::endl << std::endl;
        }
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

    bool drawMatches = false;
    if(drawMatches){
        if(!imageFilePath.empty() && !currentImagePath.empty()){
            cv::Mat imageRef = cv::imread(imageFilePath+corr_image.Name());
            cv::Mat imageCurr = cv::imread(currentImagePath);
            std::vector<cv::DMatch> matchesCV;
            std::vector<cv::KeyPoint> ptsRef;
            std::vector<cv::KeyPoint> ptsCurr;
            int num_pts = 0;
            for(size_t i = 0; i < matches.size(); i ++)
            {
                const Point2D& corr_point2D = corr_image.Point2D(matches[i].point2D_idx2);
                if (!corr_point2D.HasPoint3D()) {
                    continue;
                }
                cv::KeyPoint ptRef(corr_point2D.X(),corr_point2D.Y(), 1);
                ptsRef.push_back(ptRef);

                //cv::circle(imageRef,cv::Point(corr_point2D.X(),corr_point2D.Y()),5,
                //        cv::Scalar(255,0,255),2);

                FeatureKeypoint featurept = keypoints.at(matches[i].point2D_idx1);
                cv::KeyPoint ptCurr(featurept.x, featurept.y, 1);
                ptsCurr.push_back(ptCurr);

                //cv::circle(imageCurr,cv::Point(featurept.x, featurept.y),5,
                //        cv::Scalar(0,255,0),2);

                cv::DMatch cvmatch(num_pts, num_pts, 0.5);
                num_pts++;
                matchesCV.push_back(cvmatch);

            }         
            cv::Mat im;
            cv::drawMatches(imageRef, ptsRef, imageCurr, ptsCurr,matchesCV, im);

            cv::resize(im, im, cv::Size(im.cols/3, im.rows/3));

            cv::imshow("matches2", im);
            cv::waitKey(30);
        }
    }

    if(verboseLocal){
        std::cout << "   start 2D-3D estimation " << std::endl;
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

    Eigen::Vector4d qvec; 
    Eigen::Vector3d tvec;

    if (!EstimateAbsolutePose(abs_pose_options, tri_points2D, tri_points3D,
                            &qvec, &tvec, &cameraT, &num_inliers,
                            &inlier_mask))
    {
        if(verboseLocal){
            std::cout << " [FAILED] estimation failed." << std::endl << std::endl;
        }
        return false;
    }

    if(num_inliers < 30)
    {
        if(verboseLocal){
            std::cout << " [FAILED] not enough inlier." << num_inliers <<  std::endl << std::endl;
        }
        return false;
    }

    if (!RefineAbsolutePose(abs_pose_refinement_options, inlier_mask,
                          tri_points2D, tri_points3D, &qvec,
                          &tvec, &cameraT)) {
        if(verboseLocal){
            std::cout << " [FAILED] refinement failed." << std::endl << std::endl;
        }
        return false;
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    
    if(verboseLocal){
    std::cout << "    Result pose : " << std::endl;
    std::cout << "          q : " << qvec.transpose() << std::endl;
    std::cout << "          t : " << tvec.transpose() << std::endl;
    std::cout << "    with " << num_inliers << " inliers." << std::endl;
    std::cout << "    Time used :  " << ttrack << " second" << std::endl;
    }

    SetCurrentPose(qvec, tvec);

    return true;

}

cv::Mat Ulocalization::PoseEstimationCvMat(FeatureKeypoints &keypoints, image_t refIdx, FeatureMatches matches, double focus_length, int width, int height)
{
    bool verboseLocal = false;

    cv::Mat output = cv::Mat::zeros(4, 4, CV_32F);

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    if(verboseLocal){
        std::cout << " [POSE ESTIMATE] Pose Estimation start." << std::endl;
    }

    // create camera
    Camera cameraT;
    cameraT.SetCameraId(numCamera+1);
    cameraT.InitializeWithName("PINHOLE", focus_length, width, height);
    //std::cout << "   (fx, fx, cx, cy) : (" << cameraT.ParamsToString() << ")" << std::endl;

    //////////////////////////////////////////////////////////////////////////////
    // Search for 2D-3D correspondences
    //////////////////////////////////////////////////////////////////////////////

    // Feature match has two elements : point2D_idx1 and point2D_idx2
    Image& corr_image = reconstruction->Image(refIdx);

    if(verboseLocal){
        std::cout << "   reference image : " << corr_image.Name() << std::endl;
    }

    if (!corr_image.IsRegistered()) {
        if(verboseLocal){
            std::cout << " [FAIL] corresponding image note registered. " << std::endl << std::endl;
        }
        return output;
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

    if(verboseLocal){
        std::cout << "   start 2D-3D estimation " << std::endl;
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

    Eigen::Vector4d qvec; 
    Eigen::Vector3d tvec;
    if (!EstimateAbsolutePose(abs_pose_options, tri_points2D, tri_points3D,
                            &qvec, &tvec, &cameraT, &num_inliers,
                            &inlier_mask))
    {
        if(verboseLocal){
            std::cout << " [FAILED] estimation failed." << std::endl << std::endl;
        }
        return output;
    }

    if(num_inliers < 30){
        if(verboseLocal){
            std::cout << " [FAILED] not enough inlier." << num_inliers <<  std::endl << std::endl;
        }
        return output;
    }

    if (!RefineAbsolutePose(abs_pose_refinement_options, inlier_mask,
                          tri_points2D, tri_points3D, &qvec,
                          &tvec, &cameraT)) {
        if(verboseLocal){
            std::cout << " [FAILED] refinement failed." << std::endl << std::endl;
        }
        return output;
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    
    if(verboseLocal){
        std::cout << "    Result pose : " << std::endl;
        std::cout << "          q : " << qvec.transpose() << std::endl;
        std::cout << "          t : " << tvec.transpose() << std::endl;
        std::cout << "    with " << num_inliers << " inliers." << std::endl;
        std::cout << "    Time used :  " << ttrack << " second" << std::endl;
    }

    Eigen::Quaterniond quaternion(qvec(0),qvec(1),qvec(2),qvec(3));

    // with inverse
    Eigen::Matrix4d currentMatrix;
    Eigen::MatrixXd rotationMat = quaternion.matrix().transpose();

    tvec = - rotationMat * tvec;

    currentMatrix.block(0,0,3,3) = rotationMat;
    currentMatrix(0,3) = tvec(0);
    currentMatrix(1,3) = tvec(1);
    currentMatrix(2,3) = tvec(2);
    currentMatrix(3,0) = 0;
    currentMatrix(3,1) = 0;
    currentMatrix(3,2) = 0; 
    currentMatrix(3,3) = 1;

    //cv::Mat output(4,4, CV_32F);
    for(int i=0;i<4;i++){
        for(int j=0; j<4; j++){
            output.at<float>(i,j) = currentMatrix(i,j);
        }
    }
    return output;
}

void Ulocalization::SetCurrentPose(Eigen::Vector4d qvec, Eigen::Vector3d tvec)
{
    Eigen::Quaterniond quaternion(qvec(0),qvec(1),qvec(2),qvec(3));

    // with inverse
    Eigen::Matrix3x4d currentMatrix;
    Eigen::MatrixXd rotationMat = quaternion.matrix().transpose();
    tvec = - rotationMat * tvec;

    currentMatrix.block(0,0,3,3) = rotationMat;
    currentMatrix(0,3) = tvec(0);
    currentMatrix(1,3) = tvec(1);
    currentMatrix(2,3) = tvec(2);

    pangolin::OpenGlMatrix TwcNew;
    GetOpenGLCameraMatrix(currentMatrix, TwcNew);
    currentTwcs.push_back(TwcNew);
}

void Ulocalization::LoadFusedPointcloud(const std::string& load_path)
{
    std::cout << std::endl << "   Load fused denser point cloud. " << std::endl;
    plypoints.clear();
    plypoints = ReadPly(load_path);
    std::cout << "   There are " << plypoints.size() << " points." << std::endl << std::endl;
}




void Ulocalization::testpangolin()
{
    pangolin::CreateWindowAndBind("Colmap Ulocal: Map Viewer",1224,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(200));
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowFused("menu.Show Fused map",false,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<float> menu_sizepoint("menu. Point Size",2.0,1.0,50);
    pangolin::Var<float> menu_camerasize("menu. Camera Size",0.05,0.05,0.20);
    pangolin::Var<float> menu_cameralinewidth("menu. Camera Line Width",1.0,1.0,5.0);
    pangolin::Var<bool> menuShowPlane("menu.Show Grid Plane",true,true);
    pangolin::Var<int> menu_gridnumber("menu. Grid Element Number",25.0,3.0,40.0);
    pangolin::Var<float> menu_gridelementsize("menu. Grid Element Size",0.2,0.1,2.0);
    pangolin::Var<float> menu_gridheight("menu. Grid height",1.38,-5.0,5.0);
    pangolin::Var<bool> menuShowObject("menu.Show Object",false,true);
    pangolin::Var<float> menu_objectsize("menu. Object size",5.0,0.1,10);

    //pangolin::CreatePanel("menul").SetBounds(0.0,0.5,0.0,pangolin::Attach::Pix(200));
    pangolin::Var<bool> menu_buttonXplus("menul.X plus",false,false);
    pangolin::Var<bool> menu_buttonXminus("menur.X minus",false,false);

    pangolin::Var<bool> menu_buttonZplus("menul.Z plus",false,false);
    pangolin::Var<bool> menu_buttonZminus("menur.Z minus",false,false);

    pangolin::Var<bool> menu_buttonYplus("menul.rotate Y plus",false,false);
    pangolin::Var<bool> menu_buttonYminus("menur.rotate Y minus",false,false);

    //pangolin::CreatePanel("menur").SetBounds(0.0,
    //                       0.5,pangolin::Attach::Pix(100),pangolin::Attach::Pix(200));

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
 

    const GLfloat light_ambient[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
    const GLfloat light_diffuse[]  = { 1.0f, 1.0f, 1.0f, 1.0f };
    const GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    const GLfloat light_position[] = { 2.0f, -5.0f, 15.0f, 0.0f };

    const GLfloat mat_ambient[]    = { 0.0f, 0.0f, 0.0f, 1.0f };
    const GLfloat mat_diffuse[]    = { 0.8f, 0.8f, 0.8f, 1.0f };
    const GLfloat mat_specular[]   = { 1.0f, 1.0f, 1.0f, 1.0f };
    const GLfloat high_shininess[] = { 1.0f };

    glCullFace(GL_BACK);
    glDepthFunc(GL_LESS);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glMaterialfv(GL_FRONT, GL_AMBIENT,   mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE,   mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR,  mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);  

    while(!pangolin::ShouldQuit())
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        d_cam.Activate(s_cam);

        mKeyFrameSize = menu_camerasize;
        mCameraLineWidth = menu_cameralinewidth;
        ndivs = menu_gridnumber;
        ndivsize = menu_gridelementsize;
        height = menu_gridheight;
        mPointSize = menu_sizepoint;

        if( pangolin::Pushed(menu_buttonXplus) )
             objectX += 0.1;

        if( pangolin::Pushed(menu_buttonXminus) )
             objectX -= 0.1;

        if( pangolin::Pushed(menu_buttonZplus) )
             objectZ += 0.1;

        if( pangolin::Pushed(menu_buttonZminus) )
             objectZ -= 0.1;

        if( pangolin::Pushed(menu_buttonYplus) )
             rotateY += 0.1;

        if( pangolin::Pushed(menu_buttonYminus) )
             rotateY -= 0.1;

        //std::cout << objectX << " " << objectZ << std::endl;

        if(menuShowPoints)
        {
            // draw map points
            glPointSize(mPointSize);
            glBegin(GL_POINTS);

            for(size_t i=0, iend=mapPoints.size(); i<iend;i++)
            {
            
                float xpt = mapPoints[i].XYZ()(0);
                float ypt = mapPoints[i].XYZ()(1);
                float zpt = mapPoints[i].XYZ()(2);
                float rcolor = mapPoints[i].Color(0) / 255.0F;
                float gcolor = mapPoints[i].Color(1) / 255.0F;
                float bcolor = mapPoints[i].Color(2) / 255.0F;
                glColor3f(rcolor,gcolor,bcolor);
                glVertex3f(xpt,ypt,zpt);
            }
            glEnd();
        }

        if(menuShowFused){
            DrawFused();
        }

        if(menuShowKeyFrames)
        {
            DrawKeyframes();
        }

        if(menuShowPlane)
        {
            DrawPlane();
        }

        if(menuShowObject)
        {
            glEnable(GL_CULL_FACE);
            glEnable(GL_NORMALIZE);
            glEnable(GL_LIGHTING);
            glEnable(GL_LIGHT0);
            glEnable(GL_COLOR_MATERIAL);

            DrawTable(menu_objectsize);

            glDisable(GL_CULL_FACE);
            glDisable(GL_NORMALIZE);  
            glDisable(GL_LIGHTING);
            glDisable(GL_LIGHT0);
            glDisable(GL_COLOR_MATERIAL);
        }

        //std::cout << handler.Selected_P_w().transpose() << std::endl;

        DrawCurrentframe();

        DrawCoodinateSystem();
 
        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
}

void Ulocalization::DrawCurrentframe()
{
    const float w = mKeyFrameSize*1.5;
    const float h = w*0.75;
    const float z = w*0.6;

    for(size_t i = 0; i < currentTwcs.size(); i++){
   
    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(currentTwcs[i].m);
#else
        glMultMatrixd(currentTwcs[i].m);
#endif

    glLineWidth(mCameraLineWidth+1);
    glColor3f(1.0f,0.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);
    
    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();

    }
}

void Ulocalization::DrawKeyframes()
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;
    
    for(size_t i = 0; i < Twcs.size(); i++)
    {
        glPushMatrix();

#ifdef HAVE_GLES
            glMultMatrixf(Twcs[i].m);
#else
            glMultMatrixd(Twcs[i].m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f,1.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);
    
        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }

}

void Ulocalization::DrawCoodinateSystem()
{
    pangolin::OpenGlMatrix tempTwc;
    tempTwc.SetIdentity();

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(tempTwc.m);
#else
        glMultMatrixd(tempTwc.m);
#endif

    glLineWidth(mCameraLineWidth+2);
    glBegin(GL_LINES);
    glColor3f(1.0f,0.0f,0.0f);
    glVertex3f(0,0,0);
    glVertex3f(coordinateLength,0,0);

    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(0,0,0);
    glVertex3f(0,coordinateLength,0);

    glColor3f(0.0f,0.0f,1.0f);
    glVertex3f(0,0,0);
    glVertex3f(0,0,coordinateLength);

    glEnd();

    glPopMatrix();

}

void Ulocalization::DrawPlane()
{
    pangolin::OpenGlMatrix tempTwc;
    tempTwc.SetIdentity();

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(tempTwc.m);
#else
        glMultMatrixd(tempTwc.m);
#endif

    // Plane parallel to x-z at origin with normal -y
    const float minx = -ndivs*ndivsize;
    const float minz = -ndivs*ndivsize;
    const float maxx = ndivs*ndivsize;
    const float maxz = ndivs*ndivsize;


    glLineWidth(2);
    glColor3f(0.7f,0.7f,1.0f);
    glBegin(GL_LINES);

    for(int n = 0; n<=2*ndivs; n++)
    {
        //draw a line in x direction
        glVertex3f(minx+ndivsize*n,height,minz);
        glVertex3f(minx+ndivsize*n,height,maxz);
        //draw a line in z direction
        glVertex3f(minx,height,minz+ndivsize*n);
        glVertex3f(maxx,height,minz+ndivsize*n);
    }

    glEnd();

    glPopMatrix();

}

void Ulocalization::DrawFused()
{
    glPointSize(mPointSize);
    glBegin(GL_POINTS);

    for(size_t i=0, iend=plypoints.size(); i<iend;i++)
    {
            
        float xpt = plypoints[i].x;
        float ypt = plypoints[i].y;
        float zpt = plypoints[i].z;
        float rcolor = plypoints[i].r / 255.0F;
        float gcolor = plypoints[i].g / 255.0F;
        float bcolor = plypoints[i].b / 255.0F;
        glColor3f(rcolor,gcolor,bcolor);
        glVertex3f(xpt,ypt,zpt);
    }
    glEnd();
}


void SIFTextractionTest(const std::string& pathimg,FeatureKeypoints* keypoints,FeatureDescriptors* descriptors, int new_width, int new_height)
{

  Bitmap bitmap;
  bool tmp = bitmap.Read(pathimg, false);

  bitmap.Rescale(new_width, new_height);
  //std::cout << " bit map read test :  " << tmp << std::endl;

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  SiftExtractionOptions options;
  options.num_threads = 4;
  options.use_gpu = false;

  tmp = ExtractSiftFeaturesCPU(options, bitmap, keypoints, descriptors);
  
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

  double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

  std::cout << tmp <<  " [SIFT] sift extraction : " << keypoints->size() << " points . Used time : " << ttrack << std::endl;
  // Used time : 16.7732s for CPU to extract an image of size 3648 * 2736 .
}

bool SIFTextractionTestGPU(const std::string& pathimg,FeatureKeypoints* keypoints,FeatureDescriptors* descriptors, int new_width, int new_height)
{
    Bitmap bitmap;
    bool tmp = bitmap.Read(pathimg, false);
    /*
    // read EXIF from image file
    double focal_length = 0.0;
    if (bitmap.ExifFocalLength(&focal_length)){
      std::cout << " focal length from EXIF is : " <<  focal_length << std::endl;
    }
    std::cout << " Image size : " << bitmap.Width() << " " << bitmap.Height() << std::endl;
    */
    
    bitmap.Rescale(new_width, new_height);
    //std::cout << " bit map read test :  " << tmp << std::endl;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    SiftExtractionOptions options;
    options.num_threads = 4;
    options.use_gpu = true;

    std::unique_ptr<SiftGPU> sift_gpu;
    sift_gpu.reset(new SiftGPU);
    if (!CreateSiftGPUExtractor(options, sift_gpu.get())) {
        std::cerr << "ERROR: SiftGPU not fully supported." << std::endl;
        return false;
    }
 
    tmp = ExtractSiftFeaturesGPU(
              options, bitmap, sift_gpu.get(),
              keypoints, descriptors);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    std::cout << " [SIFT GPU] sift extraction : " << keypoints->size() << " points . Used time : " << ttrack << std::endl;
  // Used time : 0.243s for CPU to extract an image of size 3648 * 2736 .

  return tmp;
}

void GetOpenGLCameraMatrix(Eigen::Matrix3x4d matrix, pangolin::OpenGlMatrix &M)
{
    M.m[0] = matrix(0,0);
    M.m[1] = matrix(1,0);
    M.m[2] = matrix(2,0);
    M.m[3]  = 0.0;

    M.m[4] = matrix(0,1);
    M.m[5] = matrix(1,1);
    M.m[6] = matrix(2,1);
    M.m[7]  = 0.0;

    M.m[8] = matrix(0,2);
    M.m[9] = matrix(1,2);
    M.m[10] = matrix(2,2);
    M.m[11]  = 0.0;

    M.m[12] = matrix(0,3);
    M.m[13] = matrix(1,3);
    M.m[14] = matrix(2,3);
    M.m[15]  = 1.0;
}

void Ulocalization::DrawTable(float ratio){

    pangolin::OpenGlMatrix tempTwc;
    tempTwc.SetIdentity();
    tempTwc.m[12] = objectX;
    tempTwc.m[13] = height + objectBottom*ratio;
    tempTwc.m[14] = objectZ;
    tempTwc.m[0] = std::cos(rotateY);
    tempTwc.m[10] = - tempTwc.m[0];
    tempTwc.m[5] = - 1;
    tempTwc.m[8] = std::sin(rotateY);
    tempTwc.m[2] = tempTwc.m[8];

    glPushMatrix();
#ifdef HAVE_GLES
        glMultMatrixf(tempTwc.m);
#else
        glMultMatrixd(tempTwc.m);
#endif
    glColor3f(0.5, 0.25, 0.0);

    bool add_normal = true;

    for(int i = 0 ; i < numFace ; i ++){
        if(add_normal){
            glNormal3f(normals[i][0],normals[i][1],normals[i][2]); 
        }

        glBegin(GL_TRIANGLES);
        for(int j = 1; j <= indexs[i][0]; j ++){
            glVertex3f(vertex_arrayX[indexs[i][j]]*ratio, vertex_arrayY[indexs[i][j]]*ratio,
                         vertex_arrayZ[indexs[i][j]]*ratio);
        }
        glEnd();
    }
    

    bool drawLines = false;
    if(drawLines){
        glLineWidth(1);
        glBegin(GL_LINES);
        glColor3f(0.0, 0.0, 0.0);

        for(int i = 0 ; i < numFace ; i ++){
            for(int j = 1; j <= indexs[i][0]; j ++){
                glVertex3f(vertex_arrayX[indexs[i][j]]*ratio, vertex_arrayY[indexs[i][j]]*ratio,
                        vertex_arrayZ[indexs[i][j]]*ratio);
                int k = j + 1;
                if(k > 3){
                    k = 1;
                }
                glVertex3f(vertex_arrayX[indexs[i][k]]*ratio, vertex_arrayY[indexs[i][k]]*ratio,
                          vertex_arrayZ[indexs[i][k]]*ratio);
            }
        }
        glEnd();
    }

    glPopMatrix();
}

void Ulocalization::LoadPLY(const std::string& ply_path)
{
    std::cout << "   Load virtual object. " << std::endl;
    // load a mesh model from ply file
    std::ifstream fin( ply_path, std::ios_base::in );
    if ( !fin.is_open ( ) ){
        std::cout << "Cannot open file. " << std::endl;
    }
    
    std::string str;
    char ch;

    int number_col = 3;
    while ( !fin.eof() ){
        fin.get(ch);
        if( ch != ' ' && ch != '\t' && ch != '\n' ){
             str.push_back(ch);   
        }
        else
        {
            if(str == "vertex"){
                str.clear();
                getline( fin, str, '\n' ); 
                numVertex = atoi(str.c_str());       
            }
            else if(str == "face"){
                str.clear();
                getline( fin, str, '\n' );  
                numFace = atoi(str.c_str());             
            }
            else if(str == "confidence" || str == "intensity"){
                str.clear();
                number_col += 1;      
            }
            else if(str == "end_header"){
                str.clear();     
                break;     
            }
            else{
                str.clear();  
            }           
        }
    } 
    
    std::cout << "   There are " << numVertex << " vertex, and " << numFace << " Faces." << std::endl;

    vertex_arrayX = new double[numVertex];
    vertex_arrayY = new double[numVertex];
    vertex_arrayZ = new double[numVertex];

    int pos = 0;
    int counter = 0;
    double number;
    double max_edge = 0;
    //static double temp_max = 0;

    objectBottom = 10000;
    while ( !fin.eof ( ) )
    {
        fin.get ( ch );
        if( ch != ' ' && ch != '\t' && ch != '\n' )
            str.push_back ( ch );
        else{ 
            if(counter == numVertex)	break;  
            if(str == "")	        continue;
            else if(pos%number_col == 0){
                number = atof(str.c_str());               
                vertex_arrayX[counter] = number;      
            }
            else if(pos%number_col == 1){
                number = atof(str.c_str());                   
                vertex_arrayY[counter] = number;
                if(number < objectBottom)   
                     objectBottom = number;
            }
            else if(pos%number_col == 2){
                number = atof(str.c_str());                   
                vertex_arrayZ[counter] = number;  
            }

            if(pos%number_col == number_col-1){
                counter++;   
            }

            str.clear ( );    

            pos++;    
            if(abs((int)number) > max_edge) 
                max_edge = abs((int)number);      
        }   
    }
    /*
    for(int i = 0 ; i < numVertex; i ++){
        std::cout << vertex_arrayX[i] << " " << vertex_arrayY[i] << " " << vertex_arrayZ[i] << std::endl;
    }
    */

    indexs.clear();

    counter = 0;
    int i = 0;
    int point_tmp[4];
    while ( !fin.eof ( ) )
    {
        fin.get ( ch );
        if( ch != ' ' && ch != '\t' && ch != '\n' ){
            str.push_back ( ch );
        }
        else{
            if(counter == numFace)        break;  
            if(ch == '\n'){
                int* point = new int[4];
                for(int k = 0 ; k < 4 ; k++)
                    point[k] = point_tmp[k];
                //int point[4] = {point_tmp[0], point_tmp[1], point_tmp[2], point_tmp[3]};
                indexs.push_back(point);
 
                // calculate normal
                GLfloat vc1[3],vc2[3];
                GLfloat a,b,c;
                GLdouble r;                   
                vc1[0]= vertex_arrayX[point_tmp[2]] - vertex_arrayX[point_tmp[1]]; 
                vc1[1]= vertex_arrayY[point_tmp[2]] - vertex_arrayY[point_tmp[1]]; 
                vc1[2]= vertex_arrayZ[point_tmp[2]] - vertex_arrayZ[point_tmp[1]];                
                vc2[0]= vertex_arrayX[point_tmp[3]] - vertex_arrayX[point_tmp[1]]; 
                vc2[1]= vertex_arrayY[point_tmp[3]] - vertex_arrayY[point_tmp[1]]; 
                vc2[2]= vertex_arrayZ[point_tmp[3]] - vertex_arrayZ[point_tmp[1]];                
                a = vc1[1] * vc2[2] - vc2[1] * vc1[2];
                b = vc2[0] * vc1[2] - vc1[0] * vc2[2];
                c = vc1[0] * vc2[1] - vc2[0] * vc1[1];
                r = sqrt( a * a + b* b + c * c);                
                float* nor = new float[3];       
                nor[0] = a / r;
                nor[1] = b / r;
                nor[2] = c / r;
                normals.push_back(nor);

                counter++;
            }
            else if(str == "")	          continue;            
            else{                 
                point_tmp[i%4] = atoi(str.c_str());                 
                i++;              
                str.clear();       
            }         
        }
    }

    fin.close();    
    /*
    for(int j = 0 ; j < numFace; j ++){
        std::cout << j << " " << indexs[j][0] << " " <<  indexs[j][1] << " " <<  indexs[j][2] << " " << indexs[j][3] << std::endl;
    }
    */
}


void Ulocalization::SavePLYCLoud(const std::string& save_path)
{

    // save a mesh model to ply file
    std::ofstream fout( save_path, std::ios_base::out );
    fout << "ply\n";
    fout << "format ascii 1.0\n";
    fout << "element vertex " << mapPoints.size() << "\n";
    fout << "property float x\n";
    fout << "property float y\n";
    fout << "property float z\n";
    fout << "property uchar red\n";
    fout << "property uchar green\n";
    fout << "property uchar blue\n";
    fout << "end_header\n";

    for(size_t i=0, iend=mapPoints.size(); i<iend;i++)
    {
        float xpt = mapPoints[i].XYZ()(0);
        float ypt = mapPoints[i].XYZ()(1);
        float zpt = mapPoints[i].XYZ()(2);
        int rcolor = mapPoints[i].Color(0);
        int gcolor = mapPoints[i].Color(1);
        int bcolor = mapPoints[i].Color(2);
        fout << xpt << " " << ypt << " " << zpt << " " << rcolor << " " << gcolor <<  " " << bcolor << std::endl;  
    }

    fout.close();    
}



}  // namespace
