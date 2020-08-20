#include "MapTypes.h"



namespace BASTIAN
{

LKeyFrame::LKeyFrame()
{

}

void LKeyFrame::SetPose(Matrix3x4d mTwc_)
{
    mTwc = mTwc_;
    Eigen::Matrix3d rotation_mat = mTwc_.block(0,0,3,3);

    Eigen::Vector3d tvec_wc = mTwc_.block(0,3,3,1);
    Eigen::Quaterniond quaternion(rotation_mat.transpose());
    qvec_cw = quaternion;

    tvec_cw = - (qvec_cw * tvec_wc);
}

bool LKeyFrame::PosInGrid(const FeatureKeypoint &kp, int &posX, int &posY)
{
    posX = round((kp.x-0.0)*mfGridElementWidthInv);
    posY = round((kp.y-0.0)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

void LKeyFrame::AssignFeaturesToGrid()
{
    N = vFeatureKeypoints.size();

    for(int i=0;i<N;i++){
        const FeatureKeypoint &kp = vFeatureKeypoints[i];
        if(kp.x > mnMaxX)
            mnMaxX = kp.x;
        if(kp.y > mnMaxY)
            mnMaxY = kp.y;
    }
    //std::cout <<  mnMaxY << " " << mnMaxX;
    mnMaxY = mnMaxY + 10;
    mnMaxX = mnMaxX + 10;

    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY);
    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS)/(mnMaxX);

    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++){
        const FeatureKeypoint &kp = vFeatureKeypoints[i];
        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

int LKeyFrame::SharedGridWithPoints(Eigen::Quaterniond &q_cw, Eigen::Vector3d &t_cw, Eigen::Matrix3d &mCalibration)
{
    int count = 0;
    int cols_c = mCalibration(0,2) * 2;
    int rows_c = mCalibration(1,2) * 2;
    // check only the first points in each grid.
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++){
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++){
            if(mGrid[i][j].size() < 1){
                continue;
            }
            const Eigen::Vector3d &kp_w = vPoseWorld[mGrid[i][j][0]];
            Eigen::Vector3d pt_curr = q_cw * kp_w + t_cw;
            Eigen::Vector3d pt_homo = mCalibration * pt_curr;

            if(pt_homo(2) < 0)
                continue;

            Eigen::Vector2d pt_pixel(pt_homo(0)/pt_homo(2), pt_homo(1)/pt_homo(2));
            if(pt_pixel(0) < 0 || pt_pixel(0) > cols_c)
               continue;

            if(pt_pixel(1) < 0 || pt_pixel(1) > rows_c)
               continue;

            count++;
        }
    }

    return count;
}

CurrentFrame::CurrentFrame(std::vector<FeatureKeypoint> &vKeypoints_, FeatureDescriptors &mDescriptors_)
{
    SetKeypoints(vKeypoints_, mDescriptors_);
}

void CurrentFrame::SetKeypoints(std::vector<FeatureKeypoint> &vKeypoints_, FeatureDescriptors &mDescriptors_)
{

    N = vKeypoints_.size();
    vFeatureKeypoints = vKeypoints_;
    mFeatureDescriptors = mDescriptors_;

    vPointMatches = std::vector<PointMatches>(N,PointMatches());
}

void CurrentFrame::ClearMatches()
{
    last_matched_n_inliers = 0;
    vPointMatches = std::vector<PointMatches>(N,PointMatches());
}


int CurrentFrame::CountMatches()
{
    int count = 0;
    for(int i = 0; i < N; i++){
        if(vPointMatches[i].flag){
            count++;
        }
    }
    return count;
}

void CurrentFrame::SetCameraModel(float focus, int cols_, int rows_)
{
    cols = cols_;
    rows = rows_;
    mCalibration << focus, 0, cols/2,
                    0, focus, rows/2,
                    0,     0,     1;

    //cvCalibration = (cv::Mat_<double>(3, 3) << focus, 0, cols/2, 0, focus, rows/2, 0, 0, 1);
    //cvDistortionCoeff = cv::Mat::zeros(5,1,CV_32FC1);
}

bool CurrentFrame::InRange(Eigen::Vector2d pt)
{
    if(pt(0) < 0 || pt(0) > cols)
        return false;

    if(pt(1) < 0 || pt(1) > rows)
        return false;
   
    return true;
}

bool CurrentFrame::FindBestMatch(Eigen::Vector2d &pt, FeatureDescriptor &mFeatureDescriptor, 
                           FeatureKeypoint &keypoint,Eigen::Vector3d &pt_world)
{
    if(!InRange(pt))
        return false;

    for(int i = 0 ; i < N ; i++){
        float dist_x = vFeatureKeypoints[i].x - pt(0);
        float dist_y = vFeatureKeypoints[i].y - pt(1);
        if(abs(dist_x) > radius || abs(dist_y) > radius)
            continue;

        

    }
   

}



}

