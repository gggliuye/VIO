#ifndef CONFIGURATION_LY_H_
#define CONFIGURATION_LY_H_

#include <fstream>

#include <pangolin/pangolin.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "util/logging.h"
#include "util/option_manager.h"

#include "base/reconstruction.h"
#include "base/database.h"
#include "base/projection.h"
#include "base/triangulation.h"

#include "estimators/pose.h"

#include "util/bitmap.h"
#include "util/misc.h"
#include "util/bitmap.h"
#include "util/ply.h"

#include "feature/sift.h"
#include "feature/matching.h"
#include "feature/utils.h"

#include "visual_index/visual_index_liuye.h"

// include opencv libraries, only used here in the example
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

enum DeviceType
{
   SHADOW_CREATOR = 0,
   META_20 = 1
};

class TicToc{
  public:
    TicToc() {t1 = std::chrono::steady_clock::now();}
    ~TicToc(){}

    double Now() {
      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
      return std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    }

  private:
    std::chrono::steady_clock::time_point t1;
};

bool is_file_exist(const std::string fileName);

namespace Ulocal{

// structure for colmap voc tree image retrievals
struct Retrieval {
  image_t image_id = kInvalidImageId;
  std::vector<retrieval::ImageScore> image_scores;
};

// in retrieval 
//struct ImageScore {
//  int image_id = -1;
//  float score = 0.0f;
//};


}

#endif // CONFIGURATION_LY_H_
