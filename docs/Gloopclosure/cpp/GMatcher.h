#ifndef GMATCHER_H
#define GMATCHER_H

#include <fstream>
#include <iostream>
#include <vector>
#include <map>

// LoopClosure include
#include "LoopClosure/KeyFrameMapPoint.h"

void ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


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

// PKF is the old candidate keyframe
// F is the new input frame
int SearchByBoW(GKeyFrame* pKF,GKeyFrame &F, vector<GMapPoint*> &vpMapPointMatches)
{

    int HISTO_LENGTH = 30;
    int TH_LOW = 50;
    bool mbCheckOrientation = true;
    float mfNNratio = 0.9;

    // the mappoint associated with the keyframe's keypoints, and the BOW vector
    const vector<GMapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();
    const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;

    //std::cout << F.keypoints.size() << " "  << F.descriptors.cols <<" "  << F.descriptors.rows  << " \n";

    // initialize the match results
    vpMapPointMatches = vector<GMapPoint*>(F.keypoints.size(),static_cast<GMapPoint*>(NULL));


    int nmatches=0;

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

    // loop through the feature vectors of the two keyframes
    while(KFit != KFend && Fit != Fend){
        if(KFit->first == Fit->first){
            const vector<unsigned int> vIndicesKF = KFit->second;
            const vector<unsigned int> vIndicesF = Fit->second;

            // for each vector pair, loop through all their elements
            for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++) {
                // find the corresponding mappoint and its descriptor
                const unsigned int realIdxKF = vIndicesKF[iKF];
                GMapPoint* pMP = vpMapPointsKF[realIdxKF];

                if(!pMP)
                    continue;

                const cv::Mat &dKF= pKF->descriptors.row(realIdxKF);

                int bestDist1 = 256;
                int bestIdxF  = -1 ;
                int bestDist2 = 256;

                for(size_t iF=0; iF<vIndicesF.size(); iF++){
                    const unsigned int realIdxF = vIndicesF[iF];
                    // if already matched
                    if(vpMapPointMatches[realIdxF])
                        continue;

                    const cv::Mat &dF = F.descriptors.row(realIdxF);
                    const int dist =  DescriptorDistance(dKF,dF);

                    if(dist < bestDist1){
                        bestDist2 = bestDist1;
                        bestDist1 = dist;
                        bestIdxF = realIdxF;
                    } else if(dist < bestDist2) {
                        bestDist2 = dist;
                    }
                }

                // check the best match (with the lowest score)
                // need to be standout through all the keypoints -> bestdist < w*secondbestdist
                if(bestDist1<=TH_LOW){
                    if(static_cast<float>(bestDist1) < mfNNratio*static_cast<float>(bestDist2)){
                        vpMapPointMatches[bestIdxF] = pMP;
                        const cv::KeyPoint &kp = pKF->keypoints[realIdxKF];

                        if(mbCheckOrientation){
                            float rot = kp.angle-F.keypoints[bestIdxF].angle;
                            if(rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot*factor);
                            if(bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdxF);
                        }
                        nmatches++;
                    }
                }
            }
            KFit++;
            Fit++;
        } else if(KFit->first < Fit->first) {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        } else {
            Fit = F.mFeatVec.lower_bound(KFit->first);
        }
    }

    if(mbCheckOrientation){
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++){
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++){
                vpMapPointMatches[rotHist[i][j]]=static_cast<GMapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}




#endif // #ifndef GMATCHER_H
