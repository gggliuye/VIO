#include "SIFTMatcher.h"

namespace BASTIAN
{

int distance_des(FeatureDescriptor &des_1, FeatureDescriptor &des_2)
{
    //std::cout << std::endl;
    int l2_dist = 0;
    for(int i = 0; i < des_1.rows(); i++){
        int a = des_1(i); int b = des_2(i);
        //std::cout << a << "-" << b << " | ";
        l2_dist += (a-b)*(a-b);
    }
    //std::cout << sqrt(l2_dist) << std::endl;
    return sqrt(l2_dist);
}

bool MatchFrames(CurrentFrame* pCurrentFrame, LKeyFrame* pKeyFrame, double &radius)
{
/*
    Eigen::Matrix3d R_w_kf = pKeyFrame->qvec_cw.matrix().transpose();
    Eigen::Matrix3d R_curr_kf = pCurrentFrame->qvec_cw * R_w_kf;
    Eigen::Vector3d tvec_curr_kf = pCurrentFrame->tvec_cw 
                 - (pCurrentFrame->qvec_cw * (R_w_kf * pKeyFrame->tvec_cw));
*/

    Eigen::Quaterniond R_curr_w = pCurrentFrame->qvec_cw;
    Eigen::Vector3d tvec_curr_w = pCurrentFrame->tvec_cw;

    std::vector<FeatureKeypoint> &vFeatureKeypoints = pCurrentFrame->vFeatureKeypoints;
    std::vector<PointMatches> &vPointMatches = pCurrentFrame->vPointMatches;

    // find close features
    // TODO use tree structure to accelerate
    int count_kp = 0;
    int distance_threshold = 250;
    float scale_threshold = 3.0;
    float ori_threshold = 1.5;

    for(size_t i = 0; i < pKeyFrame->vPoseWorld.size(); i++){
        Eigen::Vector3d pt_curr = R_curr_w * pKeyFrame->vPoseWorld[i] + tvec_curr_w;
        Eigen::Vector3d pt_homo = pCurrentFrame->mCalibration * pt_curr;
        if(pt_homo(2) < 0)
            continue;

        Eigen::Vector2d pt_pixel(pt_homo(0)/pt_homo(2), pt_homo(1)/pt_homo(2));
        if(!pCurrentFrame->InRange(pt_pixel))
            continue;

        FeatureDescriptor &refDes = pKeyFrame->vFeatureDescriptors[i];
        // loop for close features
        int best_id = -1;
        int best_distance = 999;
        for(size_t j = 0; j < vFeatureKeypoints.size(); j ++){
            if(abs(pt_pixel(0) - vFeatureKeypoints[j].x) > radius){
                continue;
            }

            if(abs(pt_pixel(1) - vFeatureKeypoints[j].y) > radius){
                continue;
            }

            if(abs(pKeyFrame->vFeatureKeypoints[i].scale_t - vFeatureKeypoints[j].scale_t) > scale_threshold){
                continue;
            }

            if(abs(pKeyFrame->vFeatureKeypoints[i].ori_t - vFeatureKeypoints[j].ori_t) > ori_threshold){
                continue;
            }

            //Eigen::Vector<uint8_t, Eigen::Dynamic> des_diff = pCurrentFrame->mFeatureDescriptors.row(j) - refDes;
            //int norm = des_diff.norm();
            FeatureDescriptor des_cur = pCurrentFrame->mFeatureDescriptors.row(j);
            int norm = distance_des( des_cur, refDes);

            if(norm > distance_threshold){
                continue;
            }

            if(norm < best_distance){
                best_distance = norm;
                best_id = j;
            }
        }

        if(best_id > -1){
            // check if this match is better than the exist one
            if(best_distance < vPointMatches[best_id].distance){
                vPointMatches[best_id].distance = best_distance;
                vPointMatches[best_id].flag = true;
                vPointMatches[best_id].mWorld = pKeyFrame->vPoseWorld[i];
            }
            count_kp++;
        }
        
    }
    
    //std::cout << "==> Find " << count_kp << " in frame features.\n";

}


std::vector<cv::DMatch> FlannOneWay(cv::Mat &descriptor_curr, cv::Mat &descriptor_ref)
{
    cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(descriptor_curr, descriptor_ref, matches);

    double minDist = 1000;
    double maxDist = 0;
    for (int i = 0; i < descriptor_curr.rows; i++){
        double dist = matches[i].distance;
        if (dist > maxDist){
            maxDist = dist;
        }
        if (dist < minDist){
            minDist = dist;
        }
    }
    //std::cout << minDist << " " << maxDist << std::endl;

/*
    std::vector<cv::DMatch> goodMatches;
    for (int i = 0; i < descriptor_curr.rows; i++){
        double dist = matches[i].distance;
        if (dist < std::max(2.5*minDist, 0.02)){
            goodMatches.push_back(matches[i]);
        }
    }
*/
    return matches;
}

std::vector<cv::DMatch> CheckTwoWay(std::vector<cv::DMatch> &matches12, std::vector<cv::DMatch>& matches21)
{
    int n_1 = matches12.size();
    int n_2 = matches21.size();

    double minDist = 999, maxDist = 0;
    for (size_t i = 0; i < n_1; i++){
        double dist = matches12[i].distance;
        if (dist > maxDist)
            maxDist = dist;
        if (dist < minDist)
            minDist = dist;
    }

    std::vector<cv::DMatch> goodMatches;
    for(int i = 0 ; i < n_1 ;i++){
        int matched_2_idx = matches12[i].trainIdx;
        if(matches21[matched_2_idx].trainIdx == i && 
           matches12[i].distance < std::max(2.5 * minDist, 0.02)){
            goodMatches.push_back(matches12[i]);
        }
    }

    return goodMatches;
}

std::vector<cv::DMatch> MatchFramesFLANN(cv::Mat &descriptor_curr, cv::Mat &descriptor_ref)
{
    std::vector<cv::DMatch> matches12 = FlannOneWay(descriptor_curr, descriptor_ref);
    std::vector<cv::DMatch> matches21 = FlannOneWay(descriptor_ref, descriptor_curr);
    std::vector<cv::DMatch> goodMatches = CheckTwoWay(matches12, matches21);
    

    //std::cout << "==> Two-Way FLANN found " << goodMatches.size() << " good matches.\n";

    return goodMatches;
}

std::vector<cv::DMatch> MatchFramesFLANN(CurrentFrame* pCurrentFrame, LKeyFrame* pKeyFrame)
{
    cv::Mat descriptor_curr, descriptor_ref;
    
    // transform the reference keyframe descriptors
    int num_pt_ref = pKeyFrame->vFeatureKeypoints.size();
    int feature_dim = pKeyFrame->vFeatureDescriptors[0].rows();
    descriptor_ref = cv::Mat::zeros(num_pt_ref, feature_dim, CV_32FC1);
    //std::cout << descriptor_ref.cols << " * " << descriptor_ref.rows << std::endl;
    for(int i = 0; i < descriptor_ref.rows; i++){
        for(int j = 0;j < descriptor_ref.cols ; j++){
            descriptor_ref.at<float>(i,j) = pKeyFrame->vFeatureDescriptors[i](j);
        }
    }

    // transform the current keyframe descriptor
    int num_pt_curr = pCurrentFrame->mFeatureDescriptors.rows();
    descriptor_curr = cv::Mat::zeros(num_pt_curr, feature_dim, CV_32FC1);
    for(int i = 0; i < descriptor_curr.rows; i++){
        for(int j = 0;j < descriptor_curr.cols ; j++){
            descriptor_curr.at<float>(i,j) = pCurrentFrame->mFeatureDescriptors(i,j);
        }
    }

    std::vector<cv::DMatch> matches12 = FlannOneWay(descriptor_curr, descriptor_ref);
    std::vector<cv::DMatch> matches21 = FlannOneWay(descriptor_ref, descriptor_curr);
    std::vector<cv::DMatch> goodMatches = CheckTwoWay(matches12, matches21);
    

    //std::cout << "==> Two-Way FLANN found " << goodMatches.size() << " good matches.\n";

    return goodMatches;

}

}

