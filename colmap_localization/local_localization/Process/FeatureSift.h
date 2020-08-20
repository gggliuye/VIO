#ifndef LY_SIFT_FEATURE_H_
#define LY_SIFT_FEATURE_H_

#include <chrono>
#include <vector>
#include <Eigen/Core>

namespace BASTIAN
{

    typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
            FeatureDescriptors;

    typedef Eigen::Vector<uint8_t, Eigen::Dynamic> FeatureDescriptor;

    typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;

    struct FeatureKeypoint {
        FeatureKeypoint();
        FeatureKeypoint(const float x, const float y);
        FeatureKeypoint(const float x, const float y, const float scale,
                        const float orientation);
        FeatureKeypoint(const float x, const float y, const float a11,
                        const float a12, const float a21, const float a22);

        static FeatureKeypoint FromParameters(const float x, const float y,
                                              const float scale_x,
                                              const float scale_y,
                                              const float orientation,
                                              const float shear);

        void UpdateScaleAndOri();

        // Rescale the feature location and shape size by the given scale factor.
        void Rescale(const float scale);
        void Rescale(const float scale_x, const float scale_y);

        // Compute similarity shape parameters from affine shape.
        float ComputeScale() const;
        float ComputeScaleX() const;
        float ComputeScaleY() const;
        float ComputeOrientation() const;
        float ComputeShear() const;

        // Location of the feature, with the origin at the upper left image corner,
        // i.e. the upper left pixel has the coordinate (0.5, 0.5).
        float x;
        float y;

        // Affine shape of the feature.
        float a11;
        float a12;
        float a21;
        float a22;

        float scale_x_t, scale_y_t, scale_t;
        float ori_t;
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


}

#endif
