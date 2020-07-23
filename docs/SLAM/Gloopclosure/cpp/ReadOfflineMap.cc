#include "LoopClosure/ReadOfflineMap.h"

OfflineMap::OfflineMap()
{
    mMatcher  = new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(2, 20, 1));
    pORBextractor = new ORBextractor(1000,1.2,4,20,7);

    mvScaleFactors = pORBextractor->GetScaleFactors();
    mvInvScaleFactors = pORBextractor->GetInverseScaleFactors();
    mvLevelSigma2 = pORBextractor->GetScaleSigmaSquares();
    mvInvLevelSigma2 = pORBextractor->GetInverseScaleSigmaSquares();
}

void OfflineMap::InitCameraParameters(float fx_, float fy_, float cx_, float cy_)
{
    fx = fx_; fy = fy_; cx = cx_; cy = cy_;
    //init intrinsic parameters
    mIntrinsic = cv::Mat::zeros(3, 3, CV_32FC1);
    mIntrinsic.at<float>(0,0) = fx;
    mIntrinsic.at<float>(0,1) = 0;
    mIntrinsic.at<float>(0,2) = cx;
    mIntrinsic.at<float>(1,0) = 0;
    mIntrinsic.at<float>(1,1) = fy;
    mIntrinsic.at<float>(1,2) = cy;
    mIntrinsic.at<float>(2,0) = 0;
    mIntrinsic.at<float>(2,1) = 0;
    mIntrinsic.at<float>(2,2) = 1;

    mDistortionCoeff = cv::Mat::zeros(5,1,CV_32FC1);
}

void OfflineMap::LoadVocORB(const std::string &strVocFile)
{
    mpVocabulary = new ORBVocabulary();
    std::cout << "  [DBOW] load voc from " <<strVocFile<< "\n";
    mpVocabulary->loadFromBinaryFile(strVocFile);
    std::cout << "  [DBOW] load voc done\n";

    mvInvertedFile.resize(mpVocabulary->size());

    std::cout << "  [DBOW] start to index all the keyframes in database\n";

    for(int i = 0; i < keyframeSize; i++){
        vKeyFrames[i]->SetVocabulary(mpVocabulary);
        vKeyFrames[i]->ComputeBoW();
        unique_lock<mutex> lock(mMutex);
        for(DBoW2::BowVector::const_iterator vit= vKeyFrames[i]->mBowVec.begin(), 
                         vend=vKeyFrames[i]->mBowVec.end(); vit!=vend; vit++)
            mvInvertedFile[vit->first].push_back(vKeyFrames[i]);
    }

    std::cout << "  [DBOW] have set voc vector of all the keyframes in database\n";
}

bool OfflineMap::LoopDetector(cv::Mat &image)
{
    if(image.channels() != 1){
        std::cout << " [ERROR] need gray image." << std::endl;
        return false;
    }

    //std::cout << " [FBOW] Register new image. " << std::endl;
 
    GKeyFrame newImage;
    newImage.SetCamera(fx, fy, cx, cy);
    newImage.mnMaxX = image.rows;
    newImage.mnMaxY = image.cols;
    newImage.mvLevelSigma2 = pORBextractor->GetScaleSigmaSquares();

    //(*pORBextractor)(image,cv::Mat(),newImage.keypoints,newImage.descriptors);
    pORBextractor->GetFeature(image,newImage.keypoints,newImage.descriptors);

    // assign and compute the DBOW vector
    newImage.SetVocabulary(mpVocabulary);
    newImage.ComputeBoW();
    newImage.ResetmvpMapPoints();
    newImage.id = currentId + 1;
    currentId++;

    std::vector<GKeyFrame*> vpCandidateKFs = DetectRelocalizationCandidates(&newImage);

    if(vpCandidateKFs.empty())
        return false;

    int nKFs = vpCandidateKFs.size();

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<GMapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    for(int i=0; i<nKFs; i++)
    {
        GKeyFrame* pKF = vpCandidateKFs[i];
        int nmatches = SearchByBoW(pKF,newImage,vvpMapPointMatches[i]);
        if(nmatches < candidateDiscardNMatches){
            vbDiscarded[i] = true;
            continue;
        } else {
            std::cout << " [DETECTOR] BOW match number for " << pKF->id << "th keyframe : " << nmatches << std::endl;
        }
    }

    for(int i = 0; i < nKFs; i++){
        if(vbDiscarded[i])
            continue;

        // Perform 5 Ransac Iterations
        vector<bool> vbInliers;
        int nInliers;
        bool bNoMore;
        PnPsolver* pSolver = new PnPsolver(newImage,vvpMapPointMatches[i]);
        pSolver->SetRansacParameters(0.99,7,300,4,0.5,5.991);
        cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

        if(Tcw.empty())
            continue;
/*
        std::cout << i << "th candidate PnP success \n";
        std::cout << Tcw << "\n";
*/
        newImage.SetPose(Tcw);
        //Tcw.copyTo(newImage.mTcw);

        set<GMapPoint*> sFound;
        const int np = vbInliers.size();
        int nGood = 0;
        for(int j = 0; j < np ; j++){
            if(vbInliers[j]) {
                 newImage.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                 sFound.insert(vvpMapPointMatches[i][j]);
                 nGood++;
            } else
                 newImage.mvpMapPoints[j]=NULL;
        }

        // pose optimization
        // find matches again by reprojection
        GOptimization(&newImage, false);

        if(nGood > nInlierBetweenFrames)
            return true;

        int nadditional = SearchByProjection(newImage, vpCandidateKFs[i], sFound, 5, 80); 

        // optimize the pose again with the additional match point
        // the objective here is to find more match pairs
        nGood = GOptimizationRANSAC(&newImage, 3.0);

        std::cout << " Find " << nGood << " good point pairs. \n";  

        if(nGood > nInlierBetweenFrames){
            successedImages.push_back(newImage.Twc.clone());
            return true;
        }
    }


    return false;
}

std::vector<GKeyFrame*> OfflineMap::DetectRelocalizationCandidates(GKeyFrame *F)
{
    std::list<GKeyFrame*> lKFsSharingWords;
    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);
        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++){
            std::list<GKeyFrame*> &lKFs = mvInvertedFile[vit->first];
            for(std::list<GKeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++){
                GKeyFrame* pKFi=*lit;
                //std::cout << pKFi->mnRelocQuery << " " << F->id << "\n";
                if(pKFi->mnRelocQuery != F->id){
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->id;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<GKeyFrame*>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<GKeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++){
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords*0.8f;
    //list<pair<float,GKeyFrame*>> lScoreAndMatch;
    vector<GKeyFrame*> vpRelocCandidates;

    vpRelocCandidates.clear();
    // as we donnt have a covisibility graph, we limited the match part
    // TODO: keep the covisibility graph
    int nscores=0;
    float bestsiScore = 0;
    // Compute similarity score.
    for(list<GKeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++){
        GKeyFrame* pKFi = *lit;
        if(pKFi->mnRelocWords>minCommonWords){
            nscores++;
            float si = mpVocabulary->score(F->mBowVec,pKFi->mBowVec);
            if(si > bestsiScore)
                bestsiScore = si;
            //pKFi->mRelocScore=si;
            //lScoreAndMatch.push_back(make_pair(si,pKFi));
            vpRelocCandidates.push_back(pKFi);
        }
    }

    std::cout << " [DETECTOR] 1. keyframes candidates : " 
              << vpRelocCandidates.size() << "/" << keyframeSize << std::endl;
    return vpRelocCandidates;
/*

    if(lScoreAndMatch.empty())
        return vector<GKeyFrame*>();
    
    // check the neighbors(which are not candidates) of the candidate keyframe, and add the neighbors'
    // scores together to forme the score of this candidate  
    // it is the origianl process of ORBSLAM2
    list<pair<float,GKeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;
    // Lets now accumulate score by covisibility
    for(list<pair<float,GKeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        GKeyFrame* pKFi = it->second;
        vector<GKeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        GKeyFrame* pBestKF = pKFi;
        for(vector<GKeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            GKeyFrame* pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->id)
                continue;

            accScore+=pKF2->mRelocScore;
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;
    set<GKeyFrame*> spAlreadyAddedKF;

    vector<GKeyFrame*> vpRelocCandidates;

    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,GKeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        if(si>minScoreToRetain)
        {
            GKeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }
    return vpRelocCandidates;
*/
}


// TODO
int OfflineMap::SearchByProjection(GKeyFrame &F, const vector<GMapPoint*> &vpMapPoints, const float th)
{
    int nmatches = 0;
    return nmatches;
}


int OfflineMap::SearchByProjection(GKeyFrame &CurrentFrame, GKeyFrame *pKF, const set<GMapPoint*> &sAlreadyFound, const float th , const int ORBdist)
{
    int nmatches = 0;
    int HISTO_LENGTH = 30;
    bool mbCheckOrientation = true; 

    const cv::Mat Rcw = CurrentFrame.Tcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.Tcw.rowRange(0,3).col(3);
    const cv::Mat Ow = -Rcw.t() * tcw;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const vector<GMapPoint*> vpMPs = pKF->GetMapPointMatches();

    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        GMapPoint* pMP = vpMPs[i];

        if(pMP)
        {
            if(!sAlreadyFound.count(pMP))
            {
                //Project
                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw.t() + tcw;

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0 / x3Dc.at<float>(2);

                const float u = CurrentFrame.fx * xc * invzc + CurrentFrame.cx;
                const float v = CurrentFrame.fy * yc * invzc + CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

/*
                // Compute predicted scale level
                cv::Mat PO = x3Dw-Ow;
                float dist3D = cv::norm(PO);

                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();

                // Depth must be inside the scale pyramid of the image
                if(dist3D<minDistance || dist3D>maxDistance)
                    continue;

                int nPredictedLevel = pMP->PredictScale(dist3D,&CurrentFrame);

                // Search in a window
                const float radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];

                const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel-1, nPredictedLevel+1);
*/
                // TODO find the more precise predicted level (octave)
                // here we used the same level as the keyframe point
                // as in most the cases, the frames are matched with BOW only when they are very close
                // which means they will mostly have the same octave parameters
                int nPredictedLevel = pKF->keypoints[i].octave;

                const float radius = th * mvScaleFactors[nPredictedLevel];
                const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel);

                if(vIndices2.empty())
                    continue;

                const cv::Mat dMP = pKF->descriptors.row(i);

                int bestDist = 256;
                int bestIdx2 = -1;

                for(vector<size_t>::const_iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        continue;

                    const cv::Mat &d = CurrentFrame.descriptors.row(i2);

                    const int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=ORBdist)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = pKF->keypoints[i].angle-CurrentFrame.keypoints[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }

            }
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}


const int OfflineMap::TH_HIGH = 200;
const int OfflineMap::TH_MIN_MATCH_COUNT = 10;

/*
bool OfflineMap::CheckGeometry(GKeyFrame &newImage, const fbow::fBow &newimagebow, const int oldIndex)
{
    // option 1. use fundamental matrix to check, as VINS mobile did
    // option 2. project map points to try to recover pose, as ORBSLAM did
    
    // search match using BOW
    std::vector<FBOWMatch> fbowMatches = FindMatchDMatch(newImage, newimagebow, oldIndex);
    int matchsize = fbowMatches.size();


    if(matchsize < TH_MIN_MATCH_COUNT)
        return false;

    int count = 0;
//    std::vector<cv::Point3f> point3D;
//    std::vector<cv::Point2f> point2D;
    for(int i = 0; i < matchsize ; i++){
        int idmappoint = vKeyFrames[oldIndex].mpIds[fbowMatches[i].index2];
        if(idmappoint == -1){
            continue;
        }
        std::map<int, GMapPoint>::const_iterator iter = mapindices.find(idmappoint);
        if(iter != mapindices.end() ){ //&& iter->second.worldPoint.y < 1){
            //std::cout << iter->second.worldPoint.y << " ";
            point2D.push_back(newImage.keypoints[fbowMatches[i].index1].pt);
            point3D.push_back(iter->second.worldPoint);
            count++;
        }
    }

    std::cout << " [MATCH] find matches : " << count << std::endl;


    cv::Mat pnpRotationEuler;
    cv::Mat pnpTranslation;
    cv::Mat inliers;
    if(cv::solvePnPRansac(point3D, point2D, mIntrinsic, mDistortionCoeff, pnpRotationEuler, pnpTranslation,
                       false, 100, 8.0, 0.99, inliers)){
        std::cout << " [FAILED] pnp failed " << std::endl; 
        return false;
    }
    

    std::cout << " [PNP] inliers : " << inliers.rows << std::endl;
    std::cout << " [PNP] rotation    : " << pnpRotationEuler.t() << std::endl;
    std::cout << " [PNP] translation : " << pnpTranslation.t() << std::endl;
    
    return true;
}

*/

// using DMatch opencv to find match
// not used
std::vector<FBOWMatch> OfflineMap::FindMatchDMatch(GKeyFrame &newImagekf, const int oldIndex)
{
    //double score = 0;
    int count = 0;
    std::vector<FBOWMatch> fbowMatches;

    std::vector<cv::DMatch> matches;
    std::vector< std::vector<cv::DMatch> > nnMatches;

    std::vector<cv::Mat> loadedDescriptors;
    loadedDescriptors.push_back((newImagekf.descriptors));

    mMatcher->add(loadedDescriptors);
    mMatcher->knnMatch(vKeyFrames[oldIndex]->descriptors, nnMatches, 1);

    //std::cout<<" [MATCH] nnMatches size:"<<nnMatches.size()<<std::endl;

    int octave = 0;
    float pyramidThreshold;
    //int idx = 0;
    for (unsigned int i = 0; i<nnMatches.size(); ++i) {
        //std::cout << i << "th match size: "<< nnMatches[i].size() << std::endl;
        for (unsigned int j = 0; j<nnMatches[i].size(); ++j) {
            octave = newImagekf.keypoints.at(nnMatches[i][j].trainIdx).octave;

            pyramidThreshold = 100;
            if (octave <= 2)
                pyramidThreshold = 95;
            else if (octave == 3)
                pyramidThreshold = 96;
            else if (octave == 4)
                pyramidThreshold = 98;
            else if (octave >= 5)
                pyramidThreshold = 100;

            if (nnMatches[i][j].distance < pyramidThreshold) {
                FBOWMatch fbowMatch(nnMatches[i][j].queryIdx, nnMatches[i][j].trainIdx, nnMatches[i][j].distance);
                fbowMatches.push_back(fbowMatch);
                count++;
            }

        }
    }


    return fbowMatches;
}

/*
* Use the built BOW to accelerate matching process. by only match the feature points in the same node of BOW tree
* * However, the matches is extremely few, After trying to find map points, it never gets to greater than 10
* * Possible, mine fault, need to check the logical of this function
* * Should use other type of matcher at the moment
*/
//std::vector<FBOWMatch> OfflineMap::FindMatchBow(GKeyFrame &newImagekf, const fbow::fBow &newimage, const int oldIndex)


void OfflineMap::Draw()
{
    DrawTest(vKeyFrames, vMapPoints, successedImages);
}


void OfflineMap::ReadMapFromDataFile(std::string &datafile)
{
    std::ifstream inFile(datafile,std::ios::in|std::ios::binary);
    if (!inFile){
        std::cout<<" [SAVE MAP] cannot open output folder: " << datafile << std::endl;;
    }

    std::cout << std::endl << " [LOAD MAP] load map from " << datafile << std::endl;

    /////////////// step 1 ///////////////////
    // load the number of keyframes
    inFile.read((char*)&keyframeSize, sizeof(int));
    std::cout << std::endl << " [LOAD MAP] load " << keyframeSize << " keyframes. " << std::endl;

    vKeyFrames.clear();
    vKeyFrames.reserve(keyframeSize);

    //  read the {keypoints, descriptors, and ids} and keyframe {id and pose}
    for(int i=0; i < keyframeSize; i++)
    {
        GKeyFrame *pkf = new GKeyFrame;
        ReadKeyFrame(inFile, pkf);
        vKeyFrames.push_back(pkf);
    }
    std::cout << " [LOAD MAP] read all keyframes !" << std::endl;
    currentId = keyframeId;

    /////////////// step 2 ///////////////////
    // count map points
    inFile.read((char*)&countMapPoint, sizeof(int));

    vMapPoints.clear();
    vMapPoints.reserve(countMapPoint);

    std::cout << std::endl << " [LOAD MAP] load " << countMapPoint << " mappoints. " << std::endl;
    // write all the map points and its observations
    for(int i=0; i < countMapPoint; i++)
    {
        GMapPoint *pMP = new GMapPoint;
        ReadMapPoint(inFile, pMP);
        vMapPoints.push_back(pMP);
    }

    inFile.close();

    ReIndexMapPoints();

    std::cout << " [LOAD MAP] load done!" << std::endl << std::endl;
}

void OfflineMap::ReadKeyFrame(std::ifstream &inputFile, GKeyFrame *pKF)
{
    // read id
    inputFile.read((char*)&pKF->id, sizeof(int));
    if(pKF->id > keyframeId)
        keyframeId = pKF->id;
    //std::cout << "[LOAD MAP] load " << pKF->id << "th keyframe." << std::endl;


    // read pose  4*4 matrix
    int typePose;
    inputFile.read((char*)&typePose, sizeof(int));
    cv::Mat pose(4,4,typePose);
    inputFile.read((char*)pose.ptr(), pose.total() * pose.elemSize());
    pKF->SetPose(pose);

    // read keypoints
    int nkp;
    inputFile.read((char*)&nkp, sizeof(int));
    pKF->keypoints.clear();
    pKF->keypoints.reserve(nkp);
    pKF->mpIds.clear();
    pKF->mpIds.reserve(nkp);
    for (int i = 0; i < nkp; i++){
        cv::KeyPoint kp;
        inputFile.read((char*)&kp.pt.x, sizeof(float));
        inputFile.read((char*)&kp.pt.y, sizeof(float));
        inputFile.read((char*)&kp.size, sizeof(float));
        inputFile.read((char*)&kp.angle, sizeof(float));
        inputFile.read((char*)&kp.octave, sizeof(int));
        inputFile.read((char*)&kp.class_id, sizeof(int));
        int mpid = -1;
        inputFile.read((char*)&mpid, sizeof(int));
        pKF->keypoints.push_back(kp);
        pKF->mpIds.push_back(mpid);
    }

    // read descriptors
    int w ,h, typeDes;
    inputFile.read((char*)&w, sizeof(int));
    inputFile.read((char*)&h, sizeof(int));
    inputFile.read((char*)&typeDes, sizeof(int));
    cv::Mat descriptors(h,w,typeDes);
    inputFile.read((char*)descriptors.ptr(), 
                    descriptors.total() * descriptors.elemSize());
    pKF->descriptors = descriptors;
}


void OfflineMap::ReadMapPoint(std::ifstream &inputFile, GMapPoint *pMP)
{
    // read ids
    int mpid, nObs;
    inputFile.read((char*)&mpid, sizeof(int));
    // read observation times
    inputFile.read((char*)&nObs, sizeof(int));
    pMP->id = mpid;
    pMP->nObs = nObs;

    // read pose
    int typePose;
    inputFile.read((char*)&typePose, sizeof(int));
    cv::Mat pose(1,3,typePose);
    inputFile.read((char*)pose.ptr(), pose.total() * pose.elemSize());
    pMP->worldPose = pose;
    pMP->worldPoint.x = pose.at<float>(0);
    pMP->worldPoint.y = pose.at<float>(1);
    pMP->worldPoint.z = pose.at<float>(2);

    // read observations
    int observationNumber;
    inputFile.read((char*)&observationNumber, sizeof(int));
    pMP->observations.clear();
    pMP->observations.reserve(observationNumber);
    for(int i = 0; i < observationNumber; i++)
    {
        int kfid;
        inputFile.read((char*)&kfid, sizeof(int));
        pMP->observations.push_back(kfid);
    }
    //std::cout << " [LOAD MAP] read map point " << mpid << std::endl;
}

void OfflineMap::ReIndexMapPoints()
{
    for(int i = 0; i < countMapPoint; i++){
        mapindices.insert(std::map<int, GMapPoint*>::value_type(vMapPoints[i]->id, vMapPoints[i]));
    }

    for(int i = 0; i < keyframeSize ; i++){
        vKeyFrames[i]->mvpMapPoints.clear();
        vKeyFrames[i]->mvpMapPoints.reserve(vKeyFrames[i]->keypoints.size());
        for(size_t j = 0 ; j < vKeyFrames[i]->keypoints.size(); j++){
            int idmappoint = vKeyFrames[i]->mpIds[j];
            if(idmappoint == -1){
                vKeyFrames[i]->mvpMapPoints.push_back(static_cast<GMapPoint*>(NULL));
                continue;
            }
            std::map<int, GMapPoint*>::const_iterator iter = mapindices.find(idmappoint);
            if(iter != mapindices.end()){ 
                GMapPoint* pt = iter->second;
                vKeyFrames[i]->mvpMapPoints.push_back(pt);
            } else {
                vKeyFrames[i]->mvpMapPoints.push_back(static_cast<GMapPoint*>(NULL));
            }
        }
    }
}


