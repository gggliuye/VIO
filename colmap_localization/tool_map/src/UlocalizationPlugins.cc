/**
*    plugins of Ulocalization for Server Use
*
*    LIU YE @ moon light
*
*
*/


#include "UlocalizationPlugins.h"
#include <map>

//Ulocal::Ulocalization* ulocalization;
std::map<int, Ulocal::Ulocalization*> mUlocalization;

extern "C" int Internal_Map_Length()
{
    return mUlocalization.size();
}


extern "C" int Internal_Clear_Map()
{
    mUlocalization.erase(mUlocalization.begin(), mUlocalization.end());
    std::cout << " Clear Map \n";
    return 1;
}


// 1: exist ; -1 : not exist
extern "C" int Internal_Check_Key_Existance(const int key)
{
    std::map<int, Ulocal::Ulocalization*>::iterator iter;
    iter = mUlocalization.find(key);
    if(!(iter == mUlocalization.end())){
        return 1;
    }
    return -1;
}

extern "C" int Internal_Init_Ulocalization_Map(const char *database_path, const char* reconstruction_path,
                                const char* vocIndex_path, const int key)
{
//	std::cout << "Call the first version." << std::endl;
//    std::cout << "Test SIFT GPU extract features, then use Voc tree to find match image" << std::endl;
//    std::cout << "Use Pnp and optimization method to esimate camera pose." << std::endl;
//    std::cout << "    --  Test Start" << std::endl << std::endl;

    std::map<int, Ulocal::Ulocalization*>::iterator iter;
    iter = mUlocalization.find(key);
    if(!(iter == mUlocalization.end())){
        std::cout << " [ERROR INIT] already have the key !! \n";
        return -1;
    }

    std::cout << " Build object for key " << key << std::endl;

	std::string pathToDatabase(database_path); 
	std::string pathToReconstruction(reconstruction_path);
    std::string pathToVocIndex(vocIndex_path);

    //std::cout << std::endl << " UTOPA Test " << std::endl;
    //std::cout << database_path << std::endl << reconstruction_path << std::endl << vocIndex_path << std::endl;

	if (is_file_exist(pathToDatabase) && is_file_exist(pathToReconstruction) 
                         && is_file_exist(pathToVocIndex)) {
		Ulocal::Ulocalization* ulocalization = new Ulocal::Ulocalization(pathToDatabase, pathToReconstruction);

		// match using voc tree algorithm to find condidate image frame
    	ulocalization->LoadVocTree(pathToVocIndex);

    	// create sift matcher gpu
    	ulocalization->CreateGPUMatch();

        mUlocalization.insert(std::map<int, Ulocal::Ulocalization*>::value_type(key, ulocalization));

        std::cout << std::endl << " ----  [INIT DONE]  ---- " << std::endl << std::endl;

		return 1;
	}
	return -1;
}


extern "C" int Internal_Destroy_Ulocalization_Map(const int key)
{
    std::map<int, Ulocal::Ulocalization*>::iterator iter;
    //for(iter = mUlocalization.begin(); iter != mUlocalization.end(); iter++)
    iter = mUlocalization.find(key);
    if(iter == mUlocalization.end()){
        std::cout << " [ERROR DESTORY] does not find the key !! \n";
        return -1;
    }
    mUlocalization.erase(iter);
	return 1;
}

extern "C" float* Internal_Track_Ulocalization_Map(unsigned char* inputImage, int bufferLength,
                             double focus_length, int deviceType, const int key)
{
    Ulocal::Ulocalization* ulocalization;

    std::map<int, Ulocal::Ulocalization*>::iterator iter;
    iter = mUlocalization.find(key);
    if(iter == mUlocalization.end()){
        std::cout << " [ERROR TRACK] does not find the key !! \n";
        return NULL;
    }
    
    ulocalization = iter->second;

    std::cout << "image buffer length : " << bufferLength << "  focus length : " << focus_length << std::endl;
	// convert bytes to cv mat
	cv::Mat inputImageMat;
    if(deviceType == SHADOW_CREATOR)
        inputImageMat = cv::imdecode(cv::Mat(1, bufferLength, CV_8UC3, inputImage), 1);
    else if(deviceType == META_20)
        inputImageMat = cv::Mat(480, 640, CV_8UC1, inputImage);
    else {
        std::cout << "[ERROR] device type not exist ! \n";
        return NULL;
    }

    int width = inputImageMat.cols;
    int height = inputImageMat.rows;

    TicToc tictoc;

	cv::Mat mTcw = cv::Mat::zeros(0, 0, CV_32F);

    // SIFT GPU feature extraction
	FeatureKeypoints* keypoints1 = new FeatureKeypoints;
	FeatureDescriptors* descriptors1 = new FeatureDescriptors;
    cv::imwrite(std::to_string(key) + "temp.png", inputImageMat);
    if(!Ulocal::SIFTextractionTestGPU(std::to_string(key) + "temp.png",keypoints1,descriptors1,
                                           width,height)){
		std::cout << "[ERROR] fail to extract features. " << std::endl;        
    }

	Ulocal::Retrieval retrieval = ulocalization->MatchVocTreeReturnAll(*keypoints1, *descriptors1);

	int tmp = 1; 
	int maxTrival = 10;
    cv::Mat output;
    bool ifSuccess = false;
	for(auto image_score : retrieval.image_scores){
		std::cout << "Test the " << tmp++ << "th candidate: " << std::endl;
		FeatureMatches matches = ulocalization->MatchWithImageGPU(image_score.image_id, *descriptors1);
		output = ulocalization->PoseEstimationCvMat(*keypoints1, image_score.image_id, matches, 
                        focus_length, width, height);

		if(output.at<float>(3,3) > 0.5){
            ifSuccess = true;
			break;
		}
		if(tmp > maxTrival){
			std::cout << " Have made too much tests. " << std::endl << std::endl;
			std::cout << " [FAILED] Localization failed. " << std::endl;
            ifSuccess = false;
			break;
		}
	}

	std::cout  << std::endl << " [FINISH]  Total time used : " 
                   << tictoc.Now() << std::endl << std::endl;

	// change cv mat output pose to a float array as final output
	float* mModelview_matrix = CvMatToFloatArray(output, ifSuccess);
        

	return mModelview_matrix;
}




extern "C" float* CvMatToFloatArray(cv::Mat cvMatInput, bool ifSuccess)
{
	float* mModelview_matrix = new float[16];
	if (ifSuccess)
	{
		std::cout << "[INFO] SLAM has result." << std::endl;
		cv::Mat Rcw(3, 3, CV_32F);
		cv::Mat tcw(3, 1, CV_32F);

		Rcw = cvMatInput.rowRange(0, 3).colRange(0, 3).t();
		tcw = - Rcw * cvMatInput.rowRange(0, 3).col(3);

		mModelview_matrix[0] = Rcw.at<float>(0, 0);
		mModelview_matrix[1] = Rcw.at<float>(1, 0);
		mModelview_matrix[2] = Rcw.at<float>(2, 0);
		mModelview_matrix[3] = 0.0;

		mModelview_matrix[4] = Rcw.at<float>(0, 1);
		mModelview_matrix[5] = Rcw.at<float>(1, 1);
		mModelview_matrix[6] = Rcw.at<float>(2, 1);
		mModelview_matrix[7] = 0.0;

		mModelview_matrix[8] = Rcw.at<float>(0, 2);
		mModelview_matrix[9] = Rcw.at<float>(1, 2);
		mModelview_matrix[10] = Rcw.at<float>(2, 2);
		mModelview_matrix[11] = 0.0;

		mModelview_matrix[12] = tcw.at<float>(0);
		mModelview_matrix[13] = tcw.at<float>(1);
		mModelview_matrix[14] = tcw.at<float>(2);
		mModelview_matrix[15] = 1.0;

		return mModelview_matrix;
	} else {
		int i = 0;
		for (i = 0; i < 16; i++) {
			mModelview_matrix[i] = 0;
		}
		return mModelview_matrix;
	}
}
