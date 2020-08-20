#include "OrbMapping.h"



namespace BASTIAN
{

OrbMapping::OrbMapping(const std::string &map_path, const std::string &camera_cali_path)
{
    ReadMap(map_path);
    pORBextractor = new ORBextractor(1000,1.2,8,20,7);
    ReadCameraParameters(camera_cali_path);
}


OrbMapping::~OrbMapping()
{

}

bool OrbMapping::RunFeatureExtraction(const std::string &workspace_path)
{
    std::cout << "==> Read Images from : " << workspace_path << std::endl;

    for(int i = 0; i < N; i++){
        std::string image_path = workspace_path + "/" +  vColMapResults[i].path;
        //std::cout << vColMapResults[i].id << " " << image_path << std::endl;
        OrbFrame* pOrbFrame = new OrbFrame(image_path, pORBextractor, pPinholeCamera);
        pOrbFrame->mPose = vColMapResults[i].pose;

        m_orbframes.lock();
        vpOrbFrames.push_back(pOrbFrame);
        m_orbframes.unlock();

        if(i > 0 && b_show){
/*
            cv::Mat image = cv::imread(image_path);
            for(int k = 0; k < pOrbFrame->N; k++){
                cv::KeyPoint &kp = pOrbFrame->vKeyPoints[k];
                cv::circle(image, cv::Point2f(kp.pt.x, kp.pt.y), 3, cv::Scalar(255,0,0),-1);
            }
*/
            cv::Mat image_show;
            TestMatchFramesOpticalFlow(pOrbFrame, vpOrbFrames[i-1], image_show, true);

            cv::imshow("OrbFeaturesMatches", image_show);
            cv::waitKey(50);
        }

    }

}

bool OrbMapping::ReadCameraParameters(const std::string &camera_cali_path)
{
    std::cout << "==> Read Camera Intrinsic parameters from : " << camera_cali_path << std::endl;

    std::ifstream fin(camera_cali_path, std::ios_base::in );
    if ( !fin.is_open ( ) ){
        std::cout << "==> [ERROR] Cannot open camera parameter file. " << std::endl;
        return false;
    }

    char ch;
    std::string str;
    std::vector<float> vParameters;
    vParameters.reserve(4);

    while (!fin.eof( )){
        fin.get(ch);
        if( ch != ' ' && ch != '\t' && ch != '\n' ){
            str.push_back(ch);
        } else {
            float para = atof(str.c_str());
            vParameters.push_back(para);
            str.clear();
        }
    }
  
    if(vParameters.size() < 6){
        return false;
    }

    std::cout << "==> Camera Parameter : (fx, fy, cx, cy, cols, rows) : " << std::endl;
    std::cout << "         (";
    for(int i =0; i < 5; i++){
        std::cout << vParameters[i] << ", ";
    }  
    std::cout << vParameters[5] << ")" << std::endl;

    pPinholeCamera = new PinholeCamera(vParameters[0], vParameters[1], vParameters[2], 
                                       vParameters[3], vParameters[4], vParameters[5]);

    return true;
}

bool OrbMapping::ReadMap(const std::string &load_path)
{
    std::vector<std::string> vPathImages;
    std::vector<Eigen::Matrix4d> vPoseFrames;

    std::cout << "==> Read keyframe poses from : " << load_path << std::endl;

    std::ifstream fin(load_path, std::ios_base::in );
    if ( !fin.is_open ( ) ){
        std::cout << "==> [ERROR] Cannot open image path file. " << std::endl;
        return false;
    }

    vPathImages.clear();
    vPoseFrames.clear();

    std::string str;
    Eigen::Matrix4d pose_t;
    pose_t.setIdentity();

    char ch;
    bool is_name = true;
    int index = 0;
    while (!fin.eof( )){
        fin.get(ch);
        if( ch != ' ' && ch != '\t' && ch != '\n' ){
            str.push_back(ch);
        } else if (ch == '\n') {
            if(is_name){
                //std::cout << str << std::endl;
                vPathImages.push_back(str);
                is_name = false;
            }else{
                pose_t(2,3) = atof(str.c_str());
                //std::cout << pose_t << std::endl;
                vPoseFrames.push_back(pose_t);
                is_name = true;
                index = 0;
            }
            str.clear();
        } else if (ch == ' '){
            if(index == 0){
                index++;
                continue;
            }
            int i = (index-1)/4;
            int j = (index-1)%4;
            pose_t(i,j) = atof(str.c_str());
            str.clear();
            index++;
        } else {
            str.clear();
        }
    }

    vPathImages.resize(vPoseFrames.size());
    N = vPathImages.size();

    vColMapResults.reserve(N);
    for(int i = 0; i < N ; i++){
        vColMapResults.push_back(ColMapResult(vPathImages[i], vPoseFrames[i]));
    }

    // reorder the images based on its name
    std::sort(vColMapResults.begin(), vColMapResults.end(), CompareIdResult);

    std::cout << "==> Read " << vColMapResults.size() << " poses. \n";
    return true;
}

void OrbMapping::SaveMapPly(const std::string &save_path)
{
/*
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

*/
}

}
