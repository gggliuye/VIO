/**
*    plugins of Ulocalization for Server Use
*
*    LIU YE @ moon light
*
*
*/


#include "UlocalizationPluginsBeta.h"
#include <map>

//Ulocal::Ulocalization* ulocalization;
std::map<int, Ulocal::LocalizationLY*> mLocalizationLY;

extern "C" int Internal_Map_Length_Beta()
{
    return mLocalizationLY.size();
}


extern "C" int Internal_Clear_Map_Beta()
{
    mLocalizationLY.erase(mLocalizationLY.begin(), mLocalizationLY.end());
    std::cout << " Clear Map \n";
    return 1;
}


// 1: exist ; -1 : not exist
extern "C" int Internal_Check_Key_Existance_Beta(const int key)
{
    std::map<int, Ulocal::LocalizationLY*>::iterator iter;
    iter = mLocalizationLY.find(key);
    if(!(iter == mLocalizationLY.end())){
        return 1;
    }
    return -1;
}

extern "C" int Internal_Init_Ulocalization_Map_Beta(const char *database_path, const char* reconstruction_path,
                                const char* vocIndex_path, const int key)
{
//	std::cout << "Call the first version." << std::endl;
//    std::cout << "Test SIFT GPU extract features, then use Voc tree to find match image" << std::endl;
//    std::cout << "Use Pnp and optimization method to esimate camera pose." << std::endl;
//    std::cout << "    --  Test Start" << std::endl << std::endl;

    std::map<int, Ulocal::LocalizationLY*>::iterator iter;
    iter = mLocalizationLY.find(key);
    if(!(iter == mLocalizationLY.end())){
        std::cout << " [ERROR INIT] already have the key !! \n";
        return -1;
    }

    std::cout << " Build object for key : " << key << std::endl;

    std::string pathToDatabase(database_path); 
    std::string pathToReconstruction(reconstruction_path);
    std::string pathToVocIndex(vocIndex_path);

    if (is_file_exist(pathToDatabase) && is_file_exist(pathToReconstruction) 
                         && is_file_exist(pathToVocIndex)) {
        Ulocal::LocalizationLY* ulocalization = new Ulocal::LocalizationLY(pathToDatabase, pathToReconstruction, pathToVocIndex);

        mLocalizationLY.insert(std::map<int, Ulocal::LocalizationLY*>::value_type(key, ulocalization));

        std::cout << std::endl << " ----  [INIT DONE]  ---- " << std::endl << std::endl;
        return 1;
    }
    return -1;
}


extern "C" int Internal_Destroy_Ulocalization_Map_Beta(const int key)
{
    std::map<int, Ulocal::LocalizationLY*>::iterator iter;
    iter = mLocalizationLY.find(key);
    if(iter == mLocalizationLY.end()){
        std::cout << " [ERROR DESTORY] does not find the key !! \n";
        return -1;
    }
    mLocalizationLY.erase(iter);
    return 1;
}


int count = 0;
extern "C" float* Internal_Track_Ulocalization_Map_Beta(unsigned char* inputImage, int bufferLength,
                             double focus_length, int deviceType, 
                             float* init_pose, bool bUseInitGuess,
                             const int key)
{
    TicToc tictoc;
    Ulocal::LocalizationLY* ulocalization;

    std::map<int, Ulocal::LocalizationLY*>::iterator iter;
    iter = mLocalizationLY.find(key);
    if(iter == mLocalizationLY.end()){
        std::cout << " [ERROR TRACK] does not find the key !! \n";
        return NULL;
    }
    
    ulocalization = iter->second;
    std::cout << "image buffer length : " << bufferLength << "  focus length : " << focus_length << std::endl;
    
    // convert bytes to cv mat
    cv::Mat inputImageMat;
    if(deviceType == SHADOW_CREATOR){
        inputImageMat = cv::imdecode(cv::Mat(1, bufferLength, CV_8UC3, inputImage), 1);
    } else if(deviceType == META_20) {
        inputImageMat = cv::Mat(480, 640, CV_8UC1, inputImage);
        cv::cvtColor(inputImageMat,inputImageMat,cv::COLOR_GRAY2RGB);
    } else {
        std::cout << "[ERROR] device type not exist ! \n";
        return NULL;
    }

    std::cout << "   Finish decode the image. \n";

    //cv::imwrite("/data/test_" + std::to_string(count++) + ".jpg", inputImageMat);

    Eigen::Vector4d qvec(1.0,0.0,0.0,0.0);
    Eigen::Vector3d tvec(0.0,0.0,0.0);

    bool ifSuccess;

    if(bUseInitGuess){
        std::cout << " Use the user's initial pose estimation. \n";
        FloatArrayToEigenPose(qvec, tvec, init_pose);
        ifSuccess = ulocalization->LocalizeImage(inputImageMat, focus_length, qvec, tvec, true);
    } else {
        std::cout << " Do not use the user's initial pose estimation. \n";
        ifSuccess = ulocalization->LocalizeImage(inputImageMat, focus_length, qvec, tvec, false);
    }

    // change eigen output pose to a float array as final output
    float* mModelview_matrix = EigenPoseToFloatArray(qvec, tvec, ifSuccess);

    std::cout << " [TOTAL TIME USED " << tictoc.Now() << " ]\n";
    std::cout << std::endl;

    return mModelview_matrix;
}

extern "C" float* EigenPoseToFloatArray(Eigen::Vector4d &qvec, Eigen::Vector3d &tvec, bool ifSuccess)
{
    float* mModelview_matrix = new float[16];
    if (ifSuccess)
    {
        std::cout << " [INFO] SLAM has result." << std::endl;
        Eigen::Quaterniond q_eigen(qvec(0), qvec(1), qvec(2), qvec(3));
        Eigen::Matrix3d rotation_mat = q_eigen.matrix();

        mModelview_matrix[0] = rotation_mat(0, 0);
        mModelview_matrix[1] = rotation_mat(1, 0);
        mModelview_matrix[2] = rotation_mat(2, 0);
        mModelview_matrix[3] = 0.0;

        mModelview_matrix[4] = rotation_mat(0, 1);
        mModelview_matrix[5] = rotation_mat(1, 1);
        mModelview_matrix[6] = rotation_mat(2, 1);
        mModelview_matrix[7] = 0.0;

        mModelview_matrix[8] = rotation_mat(0, 2);
        mModelview_matrix[9] = rotation_mat(1, 2);
        mModelview_matrix[10] = rotation_mat(2, 2);
        mModelview_matrix[11] = 0.0;

        mModelview_matrix[12] = tvec(0);
        mModelview_matrix[13] = tvec(1);
        mModelview_matrix[14] = tvec(2);
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


extern "C" void FloatArrayToEigenPose(Eigen::Vector4d &qvec, Eigen::Vector3d &tvec, float* mModelview_matrix)
{
    Eigen::Matrix3d rotation_mat = Eigen::Matrix3d::Identity();
    rotation_mat(0, 0) = mModelview_matrix[0];
    rotation_mat(1, 0) = mModelview_matrix[1];
    rotation_mat(2, 0) = mModelview_matrix[2];

    rotation_mat(0, 1) = mModelview_matrix[4];
    rotation_mat(1, 1) = mModelview_matrix[5];
    rotation_mat(2, 1) = mModelview_matrix[6];

    rotation_mat(0, 2) = mModelview_matrix[8];
    rotation_mat(1, 2) = mModelview_matrix[9]; 
    rotation_mat(2, 2) = mModelview_matrix[10];

    Eigen::Quaterniond quaternion(rotation_mat);
    qvec << quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z();

    tvec << mModelview_matrix[12], mModelview_matrix[13], mModelview_matrix[14];

    if(false){
        std::cout << "    Inital pose : " << std::endl;
        std::cout << "          q : " << qvec.transpose() << std::endl;
        std::cout << "          t : " << tvec.transpose() << std::endl;
    }
}
