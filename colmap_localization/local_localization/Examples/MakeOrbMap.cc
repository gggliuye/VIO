#include <thread>

#include "OrbMapping.h"
#include "viewer.hpp"

using namespace BASTIAN;

std::string workspace_path;
std::shared_ptr<OrbMapping> pOrbMapping;

void Viewer()
{
    GLidarViewer(pOrbMapping);
}

void OrbMapper()
{
    pOrbMapping->RunFeatureExtraction(workspace_path);
}

int main(int argc, char **argv)
{
    if(argc != 4){
        std::cout << "./MakeOrbMap pose_path camera_parameter_file work_space_path\n";
        return 0;
    }

    pOrbMapping.reset(new OrbMapping(argv[1], argv[2]));
    workspace_path = argv[3];

    std::thread thd_Mapper(OrbMapper);
    std::thread thd_Viewer(Viewer);

    thd_Mapper.join();
    thd_Viewer.join();


    return 0;
}
