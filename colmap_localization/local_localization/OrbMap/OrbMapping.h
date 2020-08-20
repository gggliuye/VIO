#ifndef ORB_MAPPING_UTOPA_H_
#define ORB_MAPPING_UTOPA_H_

#include <fstream>
#include <iostream>
#include <mutex>

#include <string>
#include <vector>

#include "OrbFrame.h"
#include "OrbMatcher.h"

namespace BASTIAN
{

struct ColMapResult {
    ColMapResult(std::string path_, Eigen::Matrix4d pose_){
        path = path_; pose = pose_;
        std::string idName(path, 0, path.length()-4);
        id = atoi(idName.c_str());
    }

    int id;
    std::string path;
    Eigen::Matrix4d pose;
};

static bool CompareIdResult(const ColMapResult &r1, const ColMapResult &r2) {
    return r1.id < r2.id;
}

class OrbMapping
{

public:
    OrbMapping(const std::string &map_path, const std::string &camera_cali_path);
    ~OrbMapping();

public:
    bool ReadCameraParameters(const std::string &camera_cali_path);
    bool ReadMap(const std::string &map_path);
    bool RunFeatureExtraction(const std::string &workspace_path);
   

    void SaveMapPly(const std::string &save_path);

    std::vector<ColMapResult> GetColmapResults(){
        return vColMapResults;
    }

    std::vector<OrbFrame*> GetOrbFrames(){
        std::unique_lock<std::mutex> lock(m_orbframes);
        return vpOrbFrames;
    }



private:
    ORBextractor* pORBextractor;
    PinholeCamera* pPinholeCamera;

    // map data
    int N;
    std::vector<ColMapResult> vColMapResults;

    std::mutex m_orbframes;
    std::vector<OrbFrame*> vpOrbFrames;

public:
    bool b_show = true;    

    // only for debug log
    bool b_debug = false;
    std::ofstream runtimefile;
    void SetFile(std::string file_name){
        b_debug = true;
        runtimefile.open(file_name.c_str());
        runtimefile << std::fixed;
    }

};






}




#endif
