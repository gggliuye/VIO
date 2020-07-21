#include "MapSaver.h"

namespace BASTIAN
{

MapSaver::MapSaver(const std::string& database_path, const std::string& recs_path,
                         bool bViewer_)
{
    bViewer = bViewer_;

    database = new Database(database_path);
    reconstruction = new Reconstruction;
    reconstruction->ReadBinary(recs_path);

    // read the camera
    // camera = reconstruction->Camera(1);
    numCamera = reconstruction->NumCameras();

    // receive all keyframes
    framesIds.clear();
    framesIds = reconstruction->RegImageIds();

    std::cout << std::endl << " [MAP INFO] sparse feature map  " << std::endl; 
    std::cout << "            camera number : "<< reconstruction->NumCameras() << std::endl;
    std::cout << "            point number : "<< reconstruction->NumPoints3D() << std::endl;
    std::cout << "            image number : "<< reconstruction->NumImages() << std::endl << std::endl;
}

MapSaver::~MapSaver()
{
    if(database)
        database->Close();
}

void MapSaver::SaveKeyFrame(image_t framesId, std::ofstream &outputFile)
{
    if(!database->ExistsImage(framesId)){
        return;
    }

    Image& keyframe = reconstruction->Image(framesId);
    // write key frame pose
    {
        Eigen::Matrix3x4d pose = keyframe.InverseProjectionMatrix();
        // write the descriptors, which is eigen matrix type
        int rows = pose.rows(), cols = pose.cols();
        outputFile.write((char*) (&rows), sizeof(int));
        outputFile.write((char*) (&cols), sizeof(int));
        outputFile.write((char*) pose.data(), rows*cols*sizeof(double) );
    }

    std::vector<FeatureKeypoint> keypoints = database->ReadKeypoints(framesId);
    FeatureDescriptors descriptors = database->ReadDescriptors(framesId);

    // write key points
    int n_pt = keypoints.size();
    outputFile.write((char*)&n_pt, sizeof(int));

    int cols = descriptors.cols();
    outputFile.write((char*) (&cols), sizeof(int));

    //std::cout << "  ==> " << n_pt << " key points.\n";

    int pt_with_3d = 0;
    for(int i = 0; i < n_pt; i++){
        const Point2D& corr_point2D = keyframe.Point2D(i);
  
        bool has_3d = true;
        if (!corr_point2D.HasPoint3D()) {
            has_3d = false;
            outputFile.write((char*)&has_3d, sizeof(bool));
            continue;
        }

        outputFile.write((char*)&has_3d, sizeof(bool));
        const Point3D& point3D = reconstruction->Point3D(corr_point2D.Point3DId());
        {
            Eigen::Vector3d pt3d = point3D.XYZ();
            outputFile.write((char*) pt3d.data(), 3*sizeof(double) );
            //std::cout << pt3d.transpose() << std::endl;
        }

        outputFile.write((char*)&keypoints[i].x, sizeof(float));
        outputFile.write((char*)&keypoints[i].y, sizeof(float));
        outputFile.write((char*)&keypoints[i].a11, sizeof(float));
        outputFile.write((char*)&keypoints[i].a12, sizeof(float));
        outputFile.write((char*)&keypoints[i].a21, sizeof(float));
        outputFile.write((char*)&keypoints[i].a22, sizeof(float));

        // wrtie descriptor
        FeatureDescriptor_ly descriptor_i = descriptors.row(i);
        outputFile.write((char*)descriptor_i.data(), cols*sizeof(uint8_t));
        //if(pt_with_3d == 10)
        //    std::cout << descriptors.row(i) << std::endl;
        pt_with_3d++;
    }
}

void MapSaver::LoadKeyFrame(std::ifstream &inputFile)
{
    // write key frame pose
    {
        Eigen::Matrix3x4d pose;
        // write the descriptors, which is eigen matrix type
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
    std::vector<FeatureDescriptors> descriptors;
    keypoints.reserve(n_pt);
    descriptors.reserve(n_pt);

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
            //std::cout << pt3d.transpose() << std::endl;
        }

        FeatureKeypoint keypoint;
        inputFile.read((char*)&keypoint.x, sizeof(float));
        inputFile.read((char*)&keypoint.y, sizeof(float));
        inputFile.read((char*)&keypoint.a11, sizeof(float));
        inputFile.read((char*)&keypoint.a12, sizeof(float));
        inputFile.read((char*)&keypoint.a21, sizeof(float));
        inputFile.read((char*)&keypoint.a22, sizeof(float));
        keypoints.push_back(keypoint);

        // wrtie descriptor
        FeatureDescriptors descriptor_i;
        descriptor_i.resize(1, cols);
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
}

bool MapSaver::SaveMap(const std::string& save_path)
{
    // 1. number of keyframes
    // 2. Keyframes -> features 2d
    //              -> feature descriptors
    //              -> world pose 3d
    //              -> camera id
    std::ofstream outFile (save_path, std::ios::binary);
    if (!outFile){
        std::cout << " [SAVE MAP] cannot open output folder: " << save_path << std::endl;
        return false;
    }
    std::cout << " [SAVE MAP] save map to : " << save_path << std::endl;

    // write number of keyframes
    int n_kf = framesIds.size();
    outFile.write((char*)&n_kf, sizeof(int));

    // write keyframes
    for(image_t framdId : framesIds){
        SaveKeyFrame(framdId, outFile);
        //break;
    }

    outFile.close();
}

bool MapSaver::LoadMap(const std::string& load_path)
{
    std::ifstream inFile(load_path, std::ios::in|std::ios::binary);
    if (!inFile){
        std::cout << " [LOAD MAP] cannot open output folder: " << load_path << std::endl;
        return false;
    }
    std::cout << " [LOAD MAP] load map from : " << load_path << std::endl;
 
    // read number of keyframes
    int n_kf;
    inFile.read((char*)&n_kf, sizeof(int));

    std::cout << "  ==> " << n_kf << " keyframes.\n";

    for(int i = 0; i < n_kf ; i++){
        LoadKeyFrame(inFile);
        //break;
    }

    inFile.close();
}

void MapSaver::SaveKeyframesTxt(const std::string& save_path)
{
    std::cout << "  ==> save keyframes to " << save_path << std::endl;
    std::ofstream runtimefile;
    runtimefile.open(save_path.c_str());
    runtimefile << std::fixed;

    // write keyframes
    for(image_t idx : framesIds){
        if(!database->ExistsImage(idx)){
            continue;
        }

        std::string imagePath = reconstruction->Image(idx).Name();
        Eigen::Matrix3x4d pose = reconstruction->Image(idx).InverseProjectionMatrix();

        runtimefile << imagePath << std::endl;
        for(int i = 0; i < 3; i++)
            for(int j =0; j < 4; j++)
                runtimefile << " " << pose(i,j);
        runtimefile << std::endl;
    }

    runtimefile.close();
}
}
