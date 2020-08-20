#include "LocalLocalization.h"



namespace BASTIAN
{

LocalLocalization::LocalLocalization(const std::string &map_path)
{
    ReadMap(map_path);

    // using L1 normailzed descriptor (RootSIFT), which shows better property.
    // and the colmap default is L1_ROOT
    options.normalization = SiftOptions::Normalization::L1_ROOT;
}


LocalLocalization::~LocalLocalization()
{

}

// The pose of the image, defined as the transformation from world to image.
bool LocalLocalization::LocalizeImage(cv::Mat &imageGray, float focus, Eigen::Vector4d &qvec, Eigen::Vector3d &tvec)
{
    // Find close points
    Eigen::Quaterniond quaternion_cw(qvec(0), qvec(1), qvec(2), qvec(3)); 
    std::cout << "==> Initial Pose (q,t) : " << qvec.transpose() << ", " << tvec.transpose() << std::endl;
    std::vector<int> vCandidates = FindCandidateKeyFrames(quaternion_cw, tvec);
    std::cout << "==> Find " << vCandidates.size() << " nearby frames.\n";
    if(vCandidates.size() < 1){
        return false;
    }

    TicToc time_sift;
    // extract sift points 
    std::vector<FeatureKeypoint> vOutputKeypoints;
    FeatureDescriptors vOutputDescriptors;
    SIFTExtractor(imageGray, options, true, vOutputKeypoints, vOutputDescriptors);
    double time_sift_d = time_sift.Now();
    std::cout << "==> Time SIFT extraction : " << time_sift_d << std::endl;

    // create current frame object
    CurrentFrame* pCurrentFrame = new CurrentFrame(vOutputKeypoints, vOutputDescriptors);
    pCurrentFrame->SetCameraModel(focus, imageGray.cols, imageGray.rows);
    //Eigen::Quaterniond quaternion_cw(qvec(0), qvec(1), qvec(2), qvec(3)); 
    pCurrentFrame->qvec_cw = quaternion_cw;
    pCurrentFrame->tvec_cw = tvec;

    TicToc time_pose;
    double radius = 35;
    for(size_t i = 0; i < vCandidates.size(); i++){
        MatchFrames(pCurrentFrame, vLKeyFrames[vCandidates[i]], radius);
    }

    bool ret = EstimatePosePnPRansac(pCurrentFrame, qvec, tvec);
    double time_pose_d = time_pose.Now();
    std::cout << "==> Time Pose estimation : " << time_pose_d << std::endl;

    pLastFrame = pCurrentFrame;
    if(ret){
        if(b_debug){
            runtimefile << pCurrentFrame->CountMatches() << " " << time_sift_d << " " << time_pose_d;
        }
    }

    return ret;
}



bool LocalLocalization::LocalizeImageFLANN(cv::Mat &imageGray, float focus, Eigen::Vector4d &qvec, Eigen::Vector3d& tvec)
{
    CurrentFrame* pCurrentFrame = new CurrentFrame();
    pCurrentFrame->SetCameraModel(focus, imageGray.cols, imageGray.rows);

    // Find close points
    Eigen::Quaterniond quaternion_cw(qvec(0), qvec(1), qvec(2), qvec(3)); 
    std::cout << "==> Initial Pose (q,t) : " << qvec.transpose() << ", " << tvec.transpose() << std::endl;
    //std::vector<int> vCandidates = FindCandidateKeyFrames(quaternion_cw, tvec);
    std::vector<SearchResult> vCandidates = FindCandidateKeyFramesV2(quaternion_cw, tvec, pCurrentFrame->mCalibration);
    std::cout << "==> Find " << vCandidates.size() << " nearby frames.\n";
    if(vCandidates.size() < 1){
        return false;
    }

    TicToc time_sift;
    // extract sift points 
    std::vector<FeatureKeypoint> vOutputKeypoints;
    FeatureDescriptors vOutputDescriptors;
    SIFTExtractor(imageGray, options, true, vOutputKeypoints, vOutputDescriptors);
    double time_sift_d = time_sift.Now();
    std::cout << "==> Time SIFT extraction : " << time_sift_d << std::endl;

    // create current frame object
    pCurrentFrame->SetKeypoints(vOutputKeypoints, vOutputDescriptors);
    pCurrentFrame->qvec_cw = quaternion_cw;
    pCurrentFrame->tvec_cw = tvec;

    // transform the descriptor into cv::Mat format
    int num_pt_curr = pCurrentFrame->mFeatureDescriptors.rows();
    int feature_dim = pCurrentFrame->mFeatureDescriptors.cols();
    cv::Mat descriptor_curr = cv::Mat::zeros(num_pt_curr, feature_dim, CV_32FC1);
    for(int i = 0; i < descriptor_curr.rows; i++){
        for(int j = 0;j < descriptor_curr.cols ; j++){
            descriptor_curr.at<float>(i,j) = pCurrentFrame->mFeatureDescriptors(i,j);
        }
    }

    TicToc time_pose;
    int best_num_matches = 0;
    std::vector<cv::DMatch> goodMatches_best;
    int best_candidate = 0;
    int check_number = std::min(20, (int)vCandidates.size());
    for(size_t i = 0; i < check_number; i++){
        //std::cout << vCandidates[i].count << std::endl;
        if(vCandidates[i].count < 100){
            continue;
        }

        pCurrentFrame->ClearMatches();
        TicToc time_flann;
        std::vector<cv::DMatch> goodMatches = MatchFramesFLANN(descriptor_curr, vDescriptors[vCandidates[i].id]);
        //std::cout << "==> Time FLANN match : " << time_flann.Now() << std::endl;

        if(goodMatches.size() < 30){
            continue;
        }

        // assign matches
        std::vector<PointMatches> &vPointMatches = pCurrentFrame->vPointMatches;
        for(int k = 0; k < goodMatches.size(); k++){
             int best_id = goodMatches[k].queryIdx;
             vPointMatches[best_id].distance = goodMatches[k].distance;
             vPointMatches[best_id].flag = true;
             vPointMatches[best_id].mWorld =  vLKeyFrames[vCandidates[i].id]->vPoseWorld[goodMatches[k].trainIdx];
        }

        // pose estimation
        TicToc time_vali;
        Eigen::Vector4d qvec_t = qvec;
        Eigen::Vector3d tvec_t = tvec;
        bool ret = EstimatePosePnPRansac(pCurrentFrame, qvec_t, tvec_t);
        if(!ret){
            continue;
        }

        if(pCurrentFrame->last_matched_n_inliers > best_num_matches){
            best_num_matches = pCurrentFrame->last_matched_n_inliers;
            qvec = qvec_t; tvec = tvec_t;
            goodMatches_best = goodMatches;
            best_candidate = vCandidates[i].id;
        }
        
        if(best_num_matches > 80){
            break;
        }

        //std::cout << "==> Time Pose Validation : " << time_vali.Now() << std::endl;
    }
    double time_pose_d = time_pose.Now();
    std::cout << "==> Time Pose Estimation : " << time_pose_d << std::endl;


    // update for debug visualization
    if(true && best_num_matches > 20){
        pCurrentFrame->ClearMatches();
        // assign matches
        std::vector<PointMatches> &vPointMatches = pCurrentFrame->vPointMatches;
        for(int k = 0; k < goodMatches_best.size(); k++){
             int best_id = goodMatches_best[k].queryIdx;
             vPointMatches[best_id].distance = goodMatches_best[k].distance;
             vPointMatches[best_id].flag = true;
             vPointMatches[best_id].mWorld =  vLKeyFrames[best_candidate]->vPoseWorld[goodMatches_best[k].trainIdx];
        }
    }

    pLastFrame = pCurrentFrame;

    if(best_num_matches > 20){
        if(b_debug){
            runtimefile << best_num_matches << " " << time_sift_d << " " << time_pose_d;
        }

        return true;
    } else {
        return false;
    }
}

double LocalLocalization::CompareQuaternions(Eigen::Quaterniond q1, Eigen::Quaterniond q2)
{
    //Eigen::Vector3d unit_x(1.0,0.0,0.0);
    //Eigen::Quaterniond q1(qvec_1(0), qvec_1(1), qvec_1(2), qvec_1(3)); 
    Eigen::Quaterniond delta_q = q1 * q2.inverse(); 
    Eigen::Vector3d delta_euler = delta_q.matrix().eulerAngles(2,1,0);

    //std::cout << delta_q.w() << " " << delta_q.x() << std::endl;

    double delta = delta_euler.lpNorm<1>(); // norm() squaredNorm()
    return delta;
}

std::vector<int> LocalLocalization::FindCandidateKeyFrames(Eigen::Quaterniond q1, Eigen::Vector3d tvec)
{
    std::vector<int> vCandidates;
    //Eigen::Quaterniond q1(qvec(0), qvec(1), qvec(2), qvec(3));

    for(size_t i = 0; i < vLKeyFrames.size(); i++){
        Eigen::Vector3d tvec_kf = vLKeyFrames[i]->tvec_cw;
        Eigen::Quaterniond qvec_kf = vLKeyFrames[i]->qvec_cw;
        double delta_angle = CompareQuaternions(q1, qvec_kf);
        if(delta_angle < Threshold_angle){
            Eigen::Vector3d delta_distance_vec = tvec_kf - tvec;
            double delta_distance = delta_distance_vec.lpNorm<1>();
            if(delta_distance < Threshold_distance){
                vCandidates.push_back(i);
            }
        }
    }

    return vCandidates;
}

static bool CompareCandidateResult(const SearchResult &r1, const SearchResult &r2) {
    return r1.count > r2.count;
}

std::vector<SearchResult> LocalLocalization::FindCandidateKeyFramesV2(Eigen::Quaterniond q1, Eigen::Vector3d tvec, 
            Eigen::Matrix3d &mCalibration)
{
    std::vector<int> vCandidates_init = FindCandidateKeyFrames(q1, tvec);
    std::vector<SearchResult> vCandidates;
    //std::cout << std::endl;
    // further filter frames by point overlapping
    for(int i = 0;  i < vCandidates_init.size(); i++){
        LKeyFrame *pLKeyFrame = vLKeyFrames[vCandidates_init[i]];
        int count_kf = pLKeyFrame->SharedGridWithPoints(q1, tvec, mCalibration);

        //std::cout << count_kf << " ";
        vCandidates.push_back(SearchResult(vCandidates_init[i], count_kf));
    }
    //std::cout << std::endl;

    std::sort(vCandidates.begin(), vCandidates.end(), CompareCandidateResult);
    return vCandidates;
}



LKeyFrame* LocalLocalization::LoadKeyFrame(std::ifstream &inputFile)
{
    // write key frame pose
    // Compose the inverse projection matrix : from image to world space
    Matrix3x4d pose;
    {
        // read the descriptors, which is eigen matrix type
        int rows, cols;
        inputFile.read((char*) (&rows), sizeof(int));
        inputFile.read((char*) (&cols), sizeof(int));
        inputFile.read((char*) pose.data(), rows*cols*sizeof(double) );
    }

    // write key points
    int n_pt;
    inputFile.read((char*)&n_pt, sizeof(int));
 
    //std::cout << "  ==> " << n_pt << " key points.\n";

    int cols;
    inputFile.read((char*) (&cols), sizeof(int));

    std::vector<FeatureKeypoint> keypoints;
    std::vector<FeatureDescriptor> descriptors;
    std::vector<Eigen::Vector3d> vPoses;
    keypoints.reserve(n_pt);
    descriptors.reserve(n_pt);
    vPoses.reserve(n_pt);

    //int num_pt_with_3d = 0;
    for(int i = 0; i < n_pt; i++){  
        bool has_3d;
        inputFile.read((char*)&has_3d, sizeof(bool));
        if (!has_3d) {
            continue;
        }
        {
            Eigen::Vector3d pt3d;
            inputFile.read((char*) pt3d.data(), 3*sizeof(double) );
            vPoses.push_back(pt3d);
            //std::cout << pt3d.transpose() << std::endl;
        }

        FeatureKeypoint keypoint;
        inputFile.read((char*)&keypoint.x, sizeof(float));
        inputFile.read((char*)&keypoint.y, sizeof(float));
        inputFile.read((char*)&keypoint.a11, sizeof(float));
        inputFile.read((char*)&keypoint.a12, sizeof(float));
        inputFile.read((char*)&keypoint.a21, sizeof(float));
        inputFile.read((char*)&keypoint.a22, sizeof(float));
        keypoint.UpdateScaleAndOri();
        keypoints.push_back(keypoint);

        // read descriptor
        // where, descriptor_i = descriptors.row(i), which has 3d pose result.
        FeatureDescriptor descriptor_i;
        descriptor_i.resize(cols);
        inputFile.read((char*)descriptor_i.data(), cols*sizeof(uint8_t));
        descriptors.push_back(descriptor_i);
    }

    //std::cout << descriptors[10] << std::endl;

    // write the descriptors, which is eigen matrix type
/*
    {
        int rows, cols;
        inputFile.read((char*) (&rows), sizeof(int));
        inputFile.read((char*) (&cols), sizeof(int));
        descriptors.resize(rows, cols);
        inputFile.read((char*) descriptors.data(), rows*cols*sizeof(uint8_t));
    }
    //std::cout << descriptors.block(10,0,1,20) << std::endl;
*/

    // create keyframe
    LKeyFrame* pLKeyFrame = new LKeyFrame();
    pLKeyFrame->vFeatureKeypoints = keypoints;
    pLKeyFrame->vFeatureDescriptors = descriptors;
    pLKeyFrame->vPoseWorld = vPoses;
    pLKeyFrame->SetPose(pose);
    return pLKeyFrame;
}


bool LocalLocalization::ReadMap(const std::string &load_path)
{
    vLKeyFrames.clear();

    std::ifstream inFile(load_path, std::ios::in|std::ios::binary);
    if (!inFile){
        std::cout << " [LOAD MAP] cannot open output folder: " << load_path << std::endl;
        return false;
    }
    std::cout << " [LOAD MAP] load map from : " << load_path << std::endl;
 
    // read number of keyframes
    int n_kf;
    inFile.read((char*)&n_kf, sizeof(int));

    for(int i = 0; i < n_kf ; i++){
        LKeyFrame* pLKeyFrame = LoadKeyFrame(inFile);
        vLKeyFrames.push_back(pLKeyFrame);
        pLKeyFrame->AssignFeaturesToGrid();
        //break;
    }

    inFile.close();

    // transform the descriptors to cv::Mat type for FLANN process
    vDescriptors.clear();
    for(int i = 0; i < n_kf ; i++){
        LKeyFrame* pKeyFrame = vLKeyFrames[i];

        int num_pt_ref = pKeyFrame->vFeatureKeypoints.size();
        int feature_dim = pKeyFrame->vFeatureDescriptors[0].rows();
        cv::Mat descriptor_ref = cv::Mat::zeros(num_pt_ref, feature_dim, CV_32FC1);
        //std::cout << descriptor_ref.cols << " * " << descriptor_ref.rows << std::endl;
        for(int i = 0; i < descriptor_ref.rows; i++){
            for(int j = 0;j < descriptor_ref.cols ; j++){
                descriptor_ref.at<float>(i,j) = pKeyFrame->vFeatureDescriptors[i](j);
            }
        }
        vDescriptors.push_back(descriptor_ref);
    }

    std::cout << "   ==> loaded " << n_kf << " keyframes.\n";


    return true;
}

void LocalLocalization::SaveMapPly(const std::string &save_path)
{
    // count points
    int N_pts = 0;
    int N_kf = vLKeyFrames.size();

    for(int i = 0; i < N_kf; i++){
        N_pts += vLKeyFrames[i]->vFeatureKeypoints.size();
    }

    // save a mesh model to ply file
    std::ofstream fout( save_path, std::ios_base::out );
    fout << "ply\n";
    fout << "format ascii 1.0\n";
    fout << "element vertex " << N_pts << "\n";
    fout << "property float x\n";
    fout << "property float y\n";
    fout << "property float z\n";
    fout << "end_header\n";

    for(int i = 0; i < N_kf; i++){
        for(int j =0; j < vLKeyFrames[i]->vPoseWorld.size(); j++){
            Eigen::Vector3d pt_w = vLKeyFrames[i]->vPoseWorld[j];
  
            float xpt = pt_w(0);
            float ypt = pt_w(1);
            float zpt = pt_w(2);
            fout << xpt << " " << ypt << " " << zpt << std::endl;  
        }
    }

    fout.close();  


}

}
