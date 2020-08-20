#include "OrbFrame.h"


namespace BASTIAN
{

OrbFrame::OrbFrame(std::string &image_file_, ORBextractor* pORBextractor, PinholeCamera* pPinholeCamera_)
{
    image_file = image_file_;
    cv::Mat image = cv::imread(image_file_);
    cvtColor(image,image,CV_RGB2GRAY);

    pPinholeCamera = pPinholeCamera_;

    // orb feature extraction
    pORBextractor->GetFeature(image, vKeyPoints, mDescriptors);

    N = vKeyPoints.size();
    vpOrbPoints = std::vector<OrbPoint*>(N,static_cast<OrbPoint*>(NULL));

    AssignFeaturesToGrid();
}

bool OrbFrame::InBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 5;
    return BORDER_SIZE <= pt.x && pt.x < pPinholeCamera->width - BORDER_SIZE && BORDER_SIZE <= pt.y && pt.y < pPinholeCamera->height - BORDER_SIZE;
}


bool OrbFrame::PosInGrid(const cv::Point2f &kp, int &posX, int &posY)
{
    posX = round((kp.x-0.0)*mfGridElementWidthInv);
    posY = round((kp.y-0.0)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS_ORB || posY<0 || posY>=FRAME_GRID_ROWS_ORB)
        return false;

    return true;
}

void OrbFrame::AssignFeaturesToGrid()
{
    mnMaxY = pPinholeCamera->width;
    mnMaxX = pPinholeCamera->height;

    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS_ORB)/(mnMaxY);
    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS_ORB)/(mnMaxX);

    int nReserve = 0.5f*N/(FRAME_GRID_COLS_ORB*FRAME_GRID_ROWS_ORB);
    for(unsigned int i=0; i<FRAME_GRID_COLS_ORB;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS_ORB;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++){
        const cv::Point2f kp = vKeyPoints[i].pt;
        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}


std::vector<size_t> OrbFrame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel)
{
    std::vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = std::max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS_ORB)
        return vIndices;

    const int nMaxCellX = std::min((int)FRAME_GRID_COLS_ORB-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = std::max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS_ORB)
        return vIndices;

    const int nMaxCellY = std::min((int)FRAME_GRID_ROWS_ORB-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const std::vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = vKeyPoints[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}



} // namespace BASTIAN
