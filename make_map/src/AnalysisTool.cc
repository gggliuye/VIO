#include "AnalysisTool.h"

namespace Ulocal{

void AnalysisTool::ProjectMapIntoImage(cv::Mat &mImageShow, Eigen::Matrix4d ProjectionToPlane, 
                   Eigen::Vector2d vHeightRange_, float scale_, bool bSave)
{
    float scale = scale_ * resolution;
    Eigen::Vector2d vHeightRange = vHeightRange_ * resolution;

    std::vector<Eigen::Vector4d> vPoses;
    std::vector<Eigen::Vector3d> vColors;
    pLocalizationLY->GetMapPoints(vPoses, vColors);

    std::vector<Eigen::Vector3d> vT;
    std::vector<Eigen::Vector4d> vQ;
    pLocalizationLY->GetKeyFrames(vT, vQ);

    // get the 2d map size
    float max_x = 0, max_z = 0, min_x = 0, min_z = 0;
/*
    for(size_t i = 0; i < vColors.size(); i++){
        Eigen::Vector4d pojected_pt = scale * (ProjectionToPlane * vPoses[i]);
        if(pojected_pt(1) < vHeightRange(0) || pojected_pt(1) > vHeightRange(1)){
            continue;
        }
        if(pojected_pt(0) > max_x){
            max_x = pojected_pt(0);
        }
        if(pojected_pt(2) > max_z){
            max_z = pojected_pt(2);
        }
        if(pojected_pt(0) < min_x){
            min_x = pojected_pt(0);
        }
        if(pojected_pt(2) < min_z){
            min_z = pojected_pt(2);
        }
    }
*/
    for(size_t i = 0; i < vT.size(); i++){
        Eigen::Quaterniond q_cam(vQ[i](0), vQ[i](1), vQ[i](2), vQ[i](3)); 
        Eigen::Vector3d inv_t = -(q_cam.matrix().transpose() * vT[i]);

        Eigen::Vector4d camera_raw(inv_t(0), inv_t(1), inv_t(2), 1.0);
        Eigen::Vector4d pojected_pt = scale * (ProjectionToPlane * camera_raw);
        if(pojected_pt(1) < vHeightRange(0) || pojected_pt(1) > vHeightRange(1)){
            continue;
        }
        if(pojected_pt(0) > max_x){
            max_x = pojected_pt(0);
        }
        if(pojected_pt(2) > max_z){
            max_z = pojected_pt(2);
        }
        if(pojected_pt(0) < min_x){
            min_x = pojected_pt(0);
        }
        if(pojected_pt(2) < min_z){
            min_z = pojected_pt(2);
        }
    }


    float center_x = (max_x + min_x) * 0.5;
    float center_z = (max_z + min_z) * 0.5;

    // focus on the center part of image
    float original_width_x = (max_x-min_x);
    float original_width_z = (max_z-min_z);
    float re_focus = 1.4;
    min_x = center_x - original_width_x * 0.5 * re_focus;
    min_z = center_z - original_width_z * 0.5 * re_focus;

    mImageShow = cv::Mat(int(original_width_z*re_focus), int(original_width_x*re_focus), CV_8UC3, cv::Scalar(255, 255, 255));

    // key frame grid -> check for four directions
    int grid_cols = mImageShow.cols/resolution;
    int grid_rows = mImageShow.rows/resolution;
    std::vector<bool> mGrid[grid_cols][grid_rows];
    for(int i=0; i<grid_cols;i++)
        for (int j=0; j<grid_rows;j++)
            mGrid[i][j].resize(4);

    // directions grid -> check for four directions
    int grid_cols_dir = mImageShow.cols/resolution_direction;
    int grid_rows_dir = mImageShow.rows/resolution_direction;
    std::vector<bool> mGrid_Dir[grid_cols][grid_rows];
    Eigen::Vector3d mGrid_Dir_center[grid_cols][grid_rows];
    for(int i=0; i<grid_cols_dir;i++){
        for (int j=0; j<grid_rows_dir;j++){
            mGrid_Dir[i][j].resize(4);
            mGrid_Dir_center[i][j] << 0 ,0 ,0;
        }
    }

    // draw camera frames
    for(size_t i = 0; i < vT.size(); i++){
        Eigen::Quaterniond q_cam(vQ[i](0), vQ[i](1), vQ[i](2), vQ[i](3)); 
        Eigen::Vector3d inv_t = -(q_cam.matrix().transpose() * vT[i]);

        Eigen::Vector4d camera_raw(inv_t(0), inv_t(1), inv_t(2), 1.0);
        Eigen::Vector4d camera_pt = scale * (ProjectionToPlane * camera_raw);
        if(camera_pt(1) < vHeightRange(0) || camera_pt(1) > vHeightRange(1)){
            continue;
        }
        cv::Point2f pt1(camera_pt(0) - min_x, camera_pt(2) - min_z);
        cv::circle(mImageShow, pt1, 3, cv::Scalar(0,255,0), -1);

        int grid_i = int(pt1.x/resolution);
        int grid_j = int(pt1.y/resolution);

        if(grid_i < 0 || grid_i >= grid_cols || grid_j < 0 || grid_j >= grid_rows)
            continue; 

        int grid_i_dir = int(pt1.x/resolution_direction);
        int grid_j_dir = int(pt1.y/resolution_direction);
        if(grid_i_dir < 0 || grid_i_dir >= grid_cols_dir || grid_j_dir < 0 || grid_j_dir >= grid_rows_dir)
            continue; 

        // direction of the keyframe
        Eigen::Vector3d front_camera(0,0,-0.1);
        Eigen::Vector3d front_colmap = - (q_cam.matrix().transpose() * front_camera) * scale_ + inv_t;
        Eigen::Vector4d front_colmap_homo(front_colmap(0), front_colmap(1), front_colmap(2), 1.0);
        Eigen::Vector4d front_pt = scale * (ProjectionToPlane * front_colmap_homo);
        cv::Point2f pt2(front_pt(0) - min_x, front_pt(2) - min_z);
        //cv::line(mImageShow,pt1,pt2,cv::Scalar(0,50,255),1);

        cv::Point2f old_point(mGrid_Dir_center[grid_i_dir][grid_j_dir](0), mGrid_Dir_center[grid_i_dir][grid_j_dir](1));
        double old_weight = mGrid_Dir_center[grid_i_dir][grid_j_dir](2);
        cv::Point2f new_center = (old_point * old_weight + pt1) / (1 + old_weight);

        // update the grid center
        mGrid_Dir_center[grid_i_dir][grid_j_dir] << new_center.x, new_center.y , (old_weight+1);

        cv::Point2f dist_pt = pt2 - pt1;
        if(dist_pt.x > 0 && dist_pt.y > 0){
            mGrid[grid_i][grid_j][0] = true;
            mGrid_Dir[grid_i_dir][grid_j_dir][0] = true;
        }

        if(dist_pt.x > 0 && dist_pt.y < 0){
            mGrid[grid_i][grid_j][1] = true;
            mGrid_Dir[grid_i_dir][grid_j_dir][1] = true;
        }
 
        if(dist_pt.x < 0 && dist_pt.y > 0){
            mGrid[grid_i][grid_j][2] = true;
            mGrid_Dir[grid_i_dir][grid_j_dir][2] = true;
        }

        if(dist_pt.x < 0 && dist_pt.y < 0){
            mGrid[grid_i][grid_j][3] = true;
            mGrid_Dir[grid_i_dir][grid_j_dir][3] = true;
        }

    }

    // draw grid with camera frame
    for(int i=0; i<grid_cols;i++){
        for (int j=0; j<grid_rows;j++){
            int sum = 0;
            for(int k = 0; k < 4; k++)
                if(mGrid[i][j][k]){
                    sum++;
            }
            if(sum == 0)
                continue;

            cv::Point2f pt1(i*resolution, j*resolution);
            cv::Point2f pt2((i+1)*resolution, (j+1)*resolution);
            cv::Scalar color_grid = cv::Scalar(255,255,100) - cv::Scalar(50,50,0) * sum;
            //std::cout << sum << " " << color_grid << std::endl; 
            cv::rectangle(mImageShow, pt1, pt2, color_grid, -1);
        }
    }

    std::vector<cv::Point2f> vOffsets;
    vOffsets.reserve(4);
    vOffsets.push_back(cv::Point2f(1,1));
    vOffsets.push_back(cv::Point2f(1,-1));
    vOffsets.push_back(cv::Point2f(-1,1));
    vOffsets.push_back(cv::Point2f(-1,-1));


    // draw grids
    for(int i = 0; i < grid_cols ; i++){
        cv::Point2f pt1(i*resolution, 0);
        cv::Point2f pt2(i*resolution, mImageShow.rows);
        cv::line(mImageShow,pt1,pt2,cv::Scalar(0,0,0),2);
    }

    for(int i = 0; i < grid_rows ; i++){
        cv::Point2f pt1(0, i*resolution);
        cv::Point2f pt2(mImageShow.cols, i*resolution);
        cv::line(mImageShow,pt1,pt2,cv::Scalar(0,0,0),2);
    }

    // draw sparse points
    for(size_t i = 0; i < vColors.size(); i++){
        Eigen::Vector4d pojected_pt = scale * (ProjectionToPlane * vPoses[i]);
        if(pojected_pt(1) < vHeightRange(0) || pojected_pt(1) > vHeightRange(1)){
            continue;
        }
        cv::Point2f pt1(pojected_pt(0) - min_x, pojected_pt(2) - min_z);
        cv::circle(mImageShow, pt1, 1, cv::Scalar(vColors[i](0),vColors[i](1),vColors[i](2)), -1);
    }


    // draw grid with direction
    for(int i=0; i<grid_cols_dir;i++){
        for (int j=0; j<grid_rows_dir;j++){
            int sum = 0;
            for(int k = 0; k < 4; k++){
                if(mGrid_Dir[i][j][k]){
                    sum++;
                }
            }
            if(sum == 0)
                continue;

            //cv::Point2f pt1((i+0.5)*resolution_direction, (j+0.5)*resolution_direction);
            cv::Point2f pt1(mGrid_Dir_center[i][j](0), mGrid_Dir_center[i][j](1));
            for(int k = 0; k < 4; k++){
                if(!mGrid_Dir[i][j][k]){
                    cv::circle(mImageShow, pt1, 20, cv::Scalar(20,20,240), 5);
                    cv::Point2f pt2 = pt1 + vOffsets[k] * resolution_direction * 0.4;
                    cv::arrowedLine(mImageShow, pt1, pt2, cv::Scalar(20,0,200), 3);
                }
            }
        }
    }

    // filp the image
    cv::flip(mImageShow, mImageShow, 1);

    if(bSave){
        std::ofstream outFile(output_file, std::ios::binary);
        if (!outFile){
            std::cout << " [SAVE MAP] cannot open output folder: " << output_file << std::endl;
            return;
        }

        // write parameters
        outFile.write((char*)&min_x, sizeof(float));
        outFile.write((char*)&min_z, sizeof(float));
        outFile.write((char*)&resolution, sizeof(float));
        outFile.write((char*)&scale_, sizeof(float));
        outFile.write((char*)&grid_cols, sizeof(int));
        outFile.write((char*)&grid_rows, sizeof(int));

        // write the projection to plane matrix
        for(int i = 0 ; i < 4; i ++){
            for(int j = 0 ;j < 4; j++){
                float element = ProjectionToPlane(i,j);
                outFile.write((char*)&element, sizeof(float));
            }
        }

        for(int i=0; i<grid_cols;i++){
            for (int j=0; j<grid_rows;j++){
                for (int k = 0; k < 4; k++){
                    bool element = mGrid[i][j][k];
                    outFile.write((char*)&element, sizeof(bool));
                }
            }
        }
        std::cout << " [SAVE MAP] saved map to : " << output_file << std::endl;
    }
}

void AnalysisTool::DrawViewRange(cv::Mat &mImageShow, Eigen::Vector2d vPosition, Eigen::Vector2d vDirection)
{
    cv::Scalar color_max(255,100,0);


}

void AnalysisTool::TestMapRead(std::string &read_file)
{
    std::ifstream inFile(read_file, std::ios::in|std::ios::binary);
    if (!inFile){
        std::cout << " [LOAD MAP] cannot open output folder: " << read_file << std::endl;
        return;
    }

    // read number of keyframes
    float min_x, min_z, resolution, scale_;
    int grid_cols, grid_rows;

    inFile.read((char*)&min_x, sizeof(float));
    inFile.read((char*)&min_z, sizeof(float));
    inFile.read((char*)&resolution, sizeof(float));
    inFile.read((char*)&scale_, sizeof(float));
    inFile.read((char*)&grid_cols, sizeof(int));
    inFile.read((char*)&grid_rows, sizeof(int));

    float scale = scale_ * resolution;

    Eigen::Matrix4d ProjectionToPlane;
    // read the projection to plane matrix
    for(int i = 0 ; i < 4; i ++){
        for(int j = 0 ;j < 4; j++){
            float element;
            inFile.read((char*)&element, sizeof(float));
            ProjectionToPlane(i,j) = element;
        }
    }

    std::vector<bool> mGrid[grid_cols][grid_rows];
    for(int i=0; i<grid_cols;i++)
        for (int j=0; j<grid_rows;j++)
            mGrid[i][j].resize(4);

    for(int i=0; i<grid_cols;i++){
        for (int j=0; j<grid_rows;j++){
            for (int k = 0; k < 4; k++){
                bool element;
                inFile.read((char*)&element, sizeof(bool));
                mGrid[i][j][k] = element;
            }
        }
    }
    std::cout << " [LOAD MAP] loaded map from : " << read_file << std::endl;

    // show the map image
    cv::Mat mImageShow = cv::Mat(int(resolution*grid_rows), int(resolution*grid_cols), CV_8UC3, cv::Scalar(255, 255, 255));

    // draw grid with camera frame
    for(int i=0; i<grid_cols;i++){
        for (int j=0; j<grid_rows;j++){
            int sum = 0;
            for(int k = 0; k < 4; k++)
                if(mGrid[i][j][k]){
                    sum++;
            }
            if(sum == 0)
                continue;

            cv::Point2f pt1(i*resolution, j*resolution);
            cv::Point2f pt2((i+1)*resolution, (j+1)*resolution);
            cv::Scalar color_grid = cv::Scalar(255,255,100) - cv::Scalar(50,50,0) * sum;
            //std::cout << sum << " " << color_grid << std::endl; 
            cv::rectangle(mImageShow, pt1, pt2, color_grid, -1);
        }
    }
    // draw grids
    for(int i = 0; i < grid_cols+1 ; i++){
        cv::Point2f pt1(i*resolution, 0);
        cv::Point2f pt2(i*resolution, mImageShow.rows);
        cv::line(mImageShow,pt1,pt2,cv::Scalar(0,0,0),2);
    }

    for(int i = 0; i < grid_rows+1 ; i++){
        cv::Point2f pt1(0, i*resolution);
        cv::Point2f pt2(mImageShow.cols, i*resolution);
        cv::line(mImageShow,pt1,pt2,cv::Scalar(0,0,0),2);
    }

    // filp the image
    cv::flip(mImageShow, mImageShow, 1);
    cv::imwrite("MapReadImage.jpg", mImageShow);

}


} // namespace
