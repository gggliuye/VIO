/*
* Save map keyframes for local localization use
* 1. number of keyframes
* 2. Keyframes -> features 2d
*              -> feature descriptors
*              -> world pose 3d
*              -> camera id
*/



#ifndef MAP_SAVER_H_LY_
#define MAP_SAVER_H_LY_

#include <fstream>

#include "configuration.h"


namespace BASTIAN
{

using namespace colmap;

typedef Eigen::Vector<uint8_t, Eigen::Dynamic> FeatureDescriptor_ly;

class MapSaver
{

public:
    MapSaver(const std::string& database_path, const std::string& recs_path,
                       bool bViewer_=false);

    ~MapSaver();

public:
    void SaveKeyFrame(image_t framesId, std::ofstream &outputFile);
    void LoadKeyFrame(std::ifstream &inputFile);

    bool SaveMap(const std::string& save_path);
    bool LoadMap(const std::string& load_path);

    void SaveKeyframesTxt(const std::string& save_path);

private:
    bool bViewer;

    // image database
    int numImage;
    std::vector<image_t> framesIds;
    Database* database;

    // camera models
    int numCamera;

    // sparse reconstruction data
    Reconstruction* reconstruction;



};









}


#endif
