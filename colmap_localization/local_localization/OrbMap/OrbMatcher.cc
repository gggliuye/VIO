#include "OrbMatcher.h"


namespace BASTIAN
{
// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}


void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status)
{
    int j = 0;
    for (unsigned int i = 0; i < v.size(); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(std::vector<int> &v, std::vector<uchar> status)
{
    int j = 0;
    for (unsigned int i = 0; i < v.size(); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

static cv::Scalar randomColor(int64 seed)
{
    cv::RNG rng(seed);
    int icolor = (unsigned)rng;
    return cv::Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
}

cv::Mat DrawMatches(cv::Mat &image_1, cv::Mat &image_2, std::vector<cv::Point2f> &pts_1,  std::vector<cv::Point2f> &pts_2)
{
    int offset_ref_x = image_1.cols;
    int offset_ref_y = 0;
    cv::Point2f offset_ref(offset_ref_x, offset_ref_y);

    int cols_two = image_1.cols + image_2.cols;
    int rows_two = std::max(image_1.rows, image_2.rows);
    cv::Mat showImage(cv::Size(cols_two, rows_two), CV_8UC3);

    cv::Mat roi(showImage, cv::Rect(0,0,image_1.cols,image_1.rows));
    image_1.copyTo(roi);
    cv::Mat roi_ref(showImage, cv::Rect(offset_ref_x,offset_ref_y,image_2.cols,image_2.rows));
    image_2.copyTo(roi_ref);

    for(size_t i = 0; i < pts_1.size(); i++){
        cv::Point2f pt1 = pts_1[i];
        cv::Point2f pt2 = pts_2[i];
        cv::Scalar color = randomColor(cv::getTickCount());
        cv::circle(showImage, pt1, 3, color, 1);
        cv::circle(showImage, pt2+offset_ref, 3, color, 1);
        cv::line(showImage,pt1,pt2+offset_ref, color, 1);
    }
    //resize(showImage, showImage, cv::Size(showImage.cols/2, showImage.rows/2));

    // put text info
    {
        std::stringstream s;
        s << "#Matches: ";
        s << pts_1.size();
        cv::putText(showImage,s.str(),cv::Point(5,showImage.rows-5),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0,0,255),2,6);
    }
    return showImage;
}

void RejectMatchWithFundamentMatrix(std::vector<cv::Point2f> &pts_1,  std::vector<cv::Point2f> &pts_2, std::vector<int> &ids_1)
{
    std::vector<uchar> status;
    cv::findFundamentalMat(pts_1, pts_2, cv::FM_RANSAC, 1.0, 0.99, status);
    reduceVector(pts_1, status);
    reduceVector(ids_1, status);
    reduceVector(pts_2, status);
}

void RefineMatchByDescriptors(OrbFrame* pOrbFrame_1, OrbFrame* pOrbFrame_2, 
                     std::vector<cv::Point2f> &pts_1,  std::vector<cv::Point2f> &pts_2, std::vector<int> &ids_1)
{
    float radius = 3;
    int TH_HIGH = 80;
    std::vector<uchar> status = std::vector<uchar>(pts_1.size(),0);

    int n_outliers = 0;
    for(int i = 0; i < pts_1.size() ; i++){
        int nLastOctave = pOrbFrame_1->vKeyPoints[i].octave;
        std::vector<size_t> vIndices2;
        vIndices2 = pOrbFrame_2->GetFeaturesInArea(pts_2[i].x, pts_2[i].y, radius, nLastOctave-1, nLastOctave+1);

        const cv::Mat &des_1 = pOrbFrame_1->mDescriptors.row(ids_1[i]);
        
        int bestDist = 256;
        int bestIdx2 = -1;

        for(std::vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++){
            const size_t i2 = *vit;
            const cv::Mat &des_2 = pOrbFrame_2->mDescriptors.row(i2);
            const int dist = DescriptorDistance(des_1, des_2);
            if(dist<bestDist){
                bestDist=dist;
                bestIdx2=i2;
            }
        }

        if(bestDist<=TH_HIGH){
            pts_2[i] = pOrbFrame_2->vKeyPoints[bestIdx2].pt;
            status[i] = 1;
        } else {
            n_outliers++;
        }
    }

    reduceVector(pts_1, status);
    reduceVector(ids_1, status);
    reduceVector(pts_2, status);

    //std::cout << "==> Found " << n_outliers << " outliers by descriptor match." << std::endl; 
}

void TestMatchFramesFLANN(OrbFrame* pOrbFrame_1, OrbFrame* pOrbFrame_2, cv::Mat &image_show, bool bDraw)
{
    //std::vector<cv::DMatch> MatchFramesFLANN(cv::Mat &descriptor_curr, cv::Mat &descriptor_ref);
    cv::Mat mDes_1f, mDes_2f;
    pOrbFrame_1->mDescriptors.convertTo(mDes_1f, CV_32F, 1, 0);
    pOrbFrame_2->mDescriptors.convertTo(mDes_2f, CV_32F, 1, 0);
    std::vector<cv::DMatch> vDmatches = MatchFramesFLANN(mDes_1f, mDes_2f);

    if(bDraw){
        //cv::Mat image_show;
        cv::Mat image_1 = cv::imread(pOrbFrame_1->image_file);
        cv::Mat image_2 = cv::imread(pOrbFrame_2->image_file);

        cv::drawMatches(image_1, pOrbFrame_1->vKeyPoints, image_2, pOrbFrame_2->vKeyPoints,
                   vDmatches, image_show);
    }
}


void TestMatchFramesOpticalFlow(OrbFrame* pOrbFrame_1, OrbFrame* pOrbFrame_2, cv::Mat &image_show, bool bDraw)
{
    cv::Mat image_1 = cv::imread(pOrbFrame_1->image_file);
    cv::Mat image_2 = cv::imread(pOrbFrame_2->image_file);

    std::vector<cv::Point2f> cur_pts, forw_pts;
    std::vector<int> ids_1;
    cur_pts.reserve(pOrbFrame_1->N);
    for(int i = 0; i < pOrbFrame_1->N; i++){
        ids_1.push_back(i);
        cur_pts.push_back(pOrbFrame_1->vKeyPoints[i].pt);
    }
   

    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(image_1, image_2, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

    // if the points is outside the border, set the point flag to false
    for (int i = 0; i < int(forw_pts.size()); i++)
        if (status[i] && !pOrbFrame_2->InBorder(forw_pts[i]))
            status[i] = 0;

    reduceVector(ids_1, status);
    reduceVector(cur_pts, status);
    reduceVector(forw_pts, status);

    RejectMatchWithFundamentMatrix(cur_pts, forw_pts, ids_1);

    RefineMatchByDescriptors(pOrbFrame_1, pOrbFrame_2, cur_pts, forw_pts, ids_1);

    if(bDraw){
        image_show = DrawMatches(image_1, image_2, cur_pts, forw_pts);
    }

    TryTriangulation(pOrbFrame_1, pOrbFrame_2, forw_pts, ids_1);

}

bool TriangulationOnePoint(Eigen::Matrix4d &first_camera, Eigen::Matrix4d &new_camera, 
                           Eigen::Vector2d &first_keypoint, Eigen::Vector2d new_keypoint, Eigen::Vector3d &vEigenPose)
{
    double parallax = abs(new_keypoint(0) - first_keypoint(0)) + abs(new_keypoint(1) - first_keypoint(1));
    //std::cout << parallax << " ";
    if(parallax < 0.1)
        return false;

    // start triangualtion
    Eigen::Matrix4d triangulate_matrix; // = Eigen::Matrix4d::Zero();
    triangulate_matrix.row(0) = first_keypoint(0) * first_camera.row(2) - first_camera.row(0);
    triangulate_matrix.row(1) = first_keypoint(1) * first_camera.row(2) - first_camera.row(1);
    triangulate_matrix.row(2) = new_keypoint(0) * new_camera.row(2) - new_camera.row(0);
    triangulate_matrix.row(3) = new_keypoint(1) * new_camera.row(2) - new_camera.row(1);

    //std::cout << triangulate_matrix << std::endl;
    //Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::Matrix4d>(triangulate_matrix,
    //                            Eigen::ComputeThinV).matrixV().rightCols<1>();
    Eigen::Vector4d svd_V = triangulate_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    if(svd_V[3] == 0){
        return false;
    }

    Eigen::Vector4d pose_point;
    pose_point(0) = svd_V(0) / svd_V(3);
    pose_point(1) = svd_V(1) / svd_V(3);
    pose_point(2) = svd_V(2) / svd_V(3);
    pose_point(3) = 1.0;

    // check if point in front of camera
    float depth1 = first_camera.row(2) * pose_point;
    if(depth1 < 0.05){
        return false;
    }

    float depth2 = new_camera.row(2) * pose_point;
    if(depth2 < 0.05){
        return false;
    }

    vEigenPose = pose_point.segment(0,3);
    return true;
}


void TryTriangulation(OrbFrame* pOrbFrame_1, OrbFrame* pOrbFrame_2, std::vector<cv::Point2f> &pts_2, std::vector<int> &ids_1)
{
    PinholeCamera *pPinholeCamera_1 = pOrbFrame_1->pPinholeCamera;
    PinholeCamera *pPinholeCamera_2 = pOrbFrame_2->pPinholeCamera;

    std::vector<cv::KeyPoint> &vKeyPoints_1 = pOrbFrame_1->vKeyPoints;

    Eigen::Matrix4d camera_1 = pOrbFrame_1->mPose.inverse();
    Eigen::Matrix4d camera_2 = pOrbFrame_2->mPose.inverse();

    for(int i =0 ; i < ids_1.size(); i++){
        Eigen::Vector2d pt_homo_1;
        pt_homo_1(0) = (vKeyPoints_1[ids_1[i]].pt.x - pPinholeCamera_1->cx) * pPinholeCamera_1->inv_fx;
        pt_homo_1(1) = (vKeyPoints_1[ids_1[i]].pt.y - pPinholeCamera_1->cy) * pPinholeCamera_1->inv_fy;

        Eigen::Vector2d pt_homo_2;
        pt_homo_2(0) = (pts_2[i].x - pPinholeCamera_2->cx) * pPinholeCamera_2->inv_fx;
        pt_homo_2(1) = (pts_2[i].y - pPinholeCamera_2->cy) * pPinholeCamera_2->inv_fy;

        Eigen::Vector3d mWorld;
        if(TriangulationOnePoint(camera_1, camera_2, pt_homo_1, pt_homo_2, mWorld)){        
            OrbPoint *pOrbPoint = new OrbPoint(mWorld);
            pOrbFrame_1->vpOrbPoints[ids_1[i]] = pOrbPoint;
        }
    }
}



}
