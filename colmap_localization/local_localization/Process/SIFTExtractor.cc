#include <cmath>
#include "SIFTExtractor.h"

namespace BASTIAN
{
    std::vector<float> ConvertToRowMajorArrayFloat(cv::Mat image)
    {
        std::vector<float> array(image.cols * image.rows);

        uchar *imageRow;
        int index = 0;
        for (int i = 0; i < image.rows; ++i){
            imageRow = image.data + i * image.step;
            for (int j = 0; j < image.cols; ++j){
                uchar d = imageRow[j];
                //uchar d = image.at<uint8_t>(i,j);
                array[index] = static_cast<float>(d) / 255.0f;
                index++;
                //LOGI("[moonlight] sift pixel : %f\n",array[i]);
                //std::cout << (int)d << " " << array[i] << std::endl;
            }
        }
        return array;
    }

    Eigen::MatrixXf L2NormalizeFeatureDescriptors(
            const Eigen::MatrixXf& descriptors) {
        return descriptors.rowwise().normalized();
    }

    Eigen::MatrixXf L1RootNormalizeFeatureDescriptors(
            const Eigen::MatrixXf& descriptors) {
        Eigen::MatrixXf descriptors_normalized(descriptors.rows(),
                                               descriptors.cols());
        for (Eigen::MatrixXf::Index r = 0; r < descriptors.rows(); ++r) {
            const float norm = descriptors.row(r).lpNorm<1>();
            descriptors_normalized.row(r) = descriptors.row(r) / norm;
            descriptors_normalized.row(r) =
                    descriptors_normalized.row(r).array().sqrt();
        }
        return descriptors_normalized;
    }

    template <typename T1, typename T2>
    T2 TruncateCast(const T1 value) {
        return std::min(
                static_cast<T1>(std::numeric_limits<T2>::max()),
                std::max(static_cast<T1>(std::numeric_limits<T2>::min()), value));
    }

    FeatureDescriptors FeatureDescriptorsToUnsignedByte(
            const Eigen::MatrixXf& descriptors) {
        FeatureDescriptors descriptors_unsigned_byte(descriptors.rows(),
                                                     descriptors.cols());
        for (Eigen::MatrixXf::Index r = 0; r < descriptors.rows(); ++r) {
            for (Eigen::MatrixXf::Index c = 0; c < descriptors.cols(); ++c) {
                const float scaled_value = round(512.0f * descriptors(r, c));
                descriptors_unsigned_byte(r, c) =
                        TruncateCast<float, uint8_t>(scaled_value);
            }
        }
        return descriptors_unsigned_byte;
    }

    // VLFeat uses a different convention to store its descriptors. This transforms
    // the VLFeat format into the original SIFT format that is also used by SiftGPU.
    FeatureDescriptors TransformVLFeatToUBCFeatureDescriptors(
            const FeatureDescriptors& vlfeat_descriptors) {
        FeatureDescriptors ubc_descriptors(vlfeat_descriptors.rows(),
                                           vlfeat_descriptors.cols());
        const std::array<int, 8> q{{0, 7, 6, 5, 4, 3, 2, 1}};
        for (FeatureDescriptors::Index n = 0; n < vlfeat_descriptors.rows(); ++n) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    for (int k = 0; k < 8; ++k) {
                        ubc_descriptors(n, 8 * (j + 4 * i) + q[k]) =
                                vlfeat_descriptors(n, 8 * (j + 4 * i) + k);
                    }
                }
            }
        }
        return ubc_descriptors;
    }

    bool SIFTExtractor(cv::Mat &imageGray, SiftOptions &options, bool bDes,
                       std::vector<FeatureKeypoint> &vOutputKeypoints, FeatureDescriptors &vOutputDescriptors)
    {
        //ouput variables
        //std::vector<FeatureKeypoint> vOutputKeypoints;
        //FeatureDescriptors vOutputDescriptors;

        bool bCalculateDescriptors = bDes;

        // Setup SIFT extractor.
        std::unique_ptr<VlSiftFilt, void (*)(VlSiftFilt*)> sift(
                vl_sift_new(imageGray.cols, imageGray.rows, options.num_octaves,
                            options.octave_resolution, options.first_octave),
                &vl_sift_delete);
        if (!sift) {
            return false;
        }

        vl_sift_set_peak_thresh(sift.get(), options.peak_threshold);
        vl_sift_set_edge_thresh(sift.get(), options.edge_threshold);

        // Iterate through octaves.
        std::vector<size_t> level_num_features;
        std::vector<std::vector<FeatureKeypoint>> level_keypoints;
        std::vector<FeatureDescriptors> level_descriptors;
        bool first_octave = true;
        while (true) {
            if (first_octave) {
                std::vector<float> data_float = ConvertToRowMajorArrayFloat(imageGray);
                //LOGI("[moonlight] sift image size:  %d points\n",data_float.size());
                if (vl_sift_process_first_octave(sift.get(), data_float.data())) {
                    break;
                }
                first_octave = false;
            } else {
                if (vl_sift_process_next_octave(sift.get())) {
                    break;
                }
            }

            // Detect keypoints.
            vl_sift_detect(sift.get());

            // Extract detected keypoints.
            const VlSiftKeypoint* vl_keypoints = vl_sift_get_keypoints(sift.get());
            const int num_keypoints = vl_sift_get_nkeypoints(sift.get());
            //LOGI("[moonlight] sift block extracted features:  %d points\n",num_keypoints);
            //std::cout << " Layer features : " << num_keypoints << std::endl;

            if (num_keypoints == 0) {
                continue;
            }

            // Extract features with different orientations per DOG level.
            size_t level_idx = 0;
            int prev_level = -1;
            for (int i = 0; i < num_keypoints; ++i) {
                if (vl_keypoints[i].is != prev_level) {
                    if (i > 0) {
                        // Resize containers of previous DOG level.
                        level_keypoints.back().resize(level_idx);
                        if(bCalculateDescriptors){
                            level_descriptors.back().conservativeResize(level_idx, 128);
                        }
                    }

                    // Add containers for new DOG level.
                    level_idx = 0;
                    level_num_features.push_back(0);
                    level_keypoints.emplace_back(options.max_num_orientations *
                                                 num_keypoints);
                    if (bCalculateDescriptors) {
                        level_descriptors.emplace_back(
                                options.max_num_orientations * num_keypoints, 128);
                    }
                }

                level_num_features.back() += 1;
                prev_level = vl_keypoints[i].is;

                // Extract feature orientations.
                double angles[4];
                int num_orientations;
                if (options.upright) {
                    num_orientations = 1;
                    angles[0] = 0.0;
                } else {
                    num_orientations = vl_sift_calc_keypoint_orientations(
                            sift.get(), angles, &vl_keypoints[i]);
                }

                // Note that this is different from SiftGPU, which selects the top
                // global maxima as orientations while this selects the first two
                // local maxima. It is not clear which procedure is better.
                const int num_used_orientations =
                        std::min(num_orientations, options.max_num_orientations);

                for (int o = 0; o < num_used_orientations; ++o) {
                    level_keypoints.back()[level_idx] =
                            FeatureKeypoint(vl_keypoints[i].x + 0.5f, vl_keypoints[i].y + 0.5f,
                                            vl_keypoints[i].sigma, angles[o]);
                    if (bCalculateDescriptors) {
                        Eigen::MatrixXf desc(1, 128);
                        vl_sift_calc_keypoint_descriptor(sift.get(), desc.data(),
                                                         &vl_keypoints[i], angles[o]);
                        if (options.normalization ==
                                SiftOptions::Normalization::L2) {
                            desc = L2NormalizeFeatureDescriptors(desc);
                        } else if (options.normalization ==
                                SiftOptions::Normalization::L1_ROOT) {
                            desc = L1RootNormalizeFeatureDescriptors(desc);
                        } else {
                            //std::cout << "Normalization type not supported";
                        }

                        level_descriptors.back().row(level_idx) =
                                FeatureDescriptorsToUnsignedByte(desc);
                    }

                    level_idx += 1;
                }
            }

            // Resize containers for last DOG level in octave.
            level_keypoints.back().resize(level_idx);
            if (bCalculateDescriptors) {
                level_descriptors.back().conservativeResize(level_idx, 128);
            }
        }

        // Determine how many DOG levels to keep to satisfy max_num_features option.
        int first_level_to_keep = 0;
        int num_features = 0;
        int num_features_with_orientations = 0;
        for (int i = level_keypoints.size() - 1; i >= 0; --i) {
            num_features += level_num_features[i];
            num_features_with_orientations += level_keypoints[i].size();
            if (num_features > options.max_num_features) {
                first_level_to_keep = i;
                break;
            }
        }

        // Extract the features to be kept.
        {
            size_t k = 0;
            vOutputKeypoints.resize(num_features_with_orientations);
            for (size_t i = first_level_to_keep; i < level_keypoints.size(); ++i) {
                for (size_t j = 0; j < level_keypoints[i].size(); ++j) {
                    vOutputKeypoints[k] = level_keypoints[i][j];
                    k += 1;
                }
            }
        }
        // Compute the descriptors for the detected keypoints.
        if (bCalculateDescriptors) {
            size_t k = 0;
            vOutputDescriptors.resize(num_features_with_orientations, 128);
            for (size_t i = first_level_to_keep; i < level_keypoints.size(); ++i) {
                for (size_t j = 0; j < level_keypoints[i].size(); ++j) {
                    vOutputDescriptors.row(k) = level_descriptors[i].row(j);
                    k += 1;
                }
            }
            vOutputDescriptors = TransformVLFeatToUBCFeatureDescriptors(vOutputDescriptors);
        }

        return true;
    }

} // namespace
