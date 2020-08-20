cmake_minimum_required(VERSION 2.8)
project(local_localization)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native -faligned-new")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.4 QUIET)

# find pangolin visualization only for build the demo (the main scripts won't use pangolin)
find_package(Pangolin REQUIRED)

find_package(Ceres REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/Process
${PROJECT_SOURCE_DIR}/OrbMap
${PROJECT_SOURCE_DIR}/VLFeat

${EIGEN3_INCLUDE_DIR}
${CERES_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/lib)

add_library(${PROJECT_NAME} SHARED
        VLFeat/aib.c
        VLFeat/array.c
        VLFeat/covdet.c
        VLFeat/dsift.c
        VLFeat/fisher.c
        VLFeat/generic.c
        VLFeat/getopt_long.c
        VLFeat/gmm.c
        VLFeat/hikmeans.c
        VLFeat/hog.c
        VLFeat/homkermap.c
        VLFeat/ikmeans.c
        VLFeat/ikmeans_elkan.tc
        VLFeat/ikmeans_init.tc
        VLFeat/ikmeans_lloyd.tc
        VLFeat/imopv.c
        #src/main/cpp/VLFeat/imopv_sse2.c
        VLFeat/kdtree.c
        VLFeat/kmeans.c
        VLFeat/host.c
        VLFeat/lbp.c
        VLFeat/liop.c
        VLFeat/mathop.c
        #src/main/cpp/VLFeat/mathop_avx.c
        #src/main/cpp/VLFeat/mathop_sse2.c
        VLFeat/mser.c
        VLFeat/pgm.c
        VLFeat/quickshift.c
        VLFeat/random.c
        VLFeat/rodrigues.c
        VLFeat/scalespace.c
        VLFeat/sift.c
        VLFeat/slic.c
        VLFeat/stringop.c
        VLFeat/svm.c
        VLFeat/svmdataset.c
        VLFeat/vlad.c

        Process/SIFTExtractor.cc
        Process/FeatureSift.cc
        Process/SIFTMatcher.cc
        Process/LocalLocalization.cc
        Process/MapTypes.cc
        Process/PoseEstimation.cc
        Process/Factor.cc

        OrbMap/OrbMapping.cc
        OrbMap/orbextractor.cc
        OrbMap/OrbFrame.cc
        OrbMap/OrbPoint.cc
        OrbMap/OrbMatcher.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${CERES_LIBRARIES}
)

add_definitions("-DVL_DISABLE_AVX")
add_definitions("-DVL_DISABLE_SSE2")
if(NOT OPENMP_ENABLED OR NOT OPENMP_FOUND)
    add_definitions("-DVL_DISABLE_OPENMP")
endif()

# Build examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/bin)

add_executable(test_sift
Examples/test_sift.cc)
target_link_libraries(test_sift ${PROJECT_NAME})

add_executable(Test_video_images
Examples/Test_video_images.cc)
target_link_libraries(Test_video_images ${PROJECT_NAME})

add_executable(MakeOrbMap
Examples/MakeOrbMap.cc
OrbMap/viewer.hpp
${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(MakeOrbMap 
${PROJECT_NAME}
${Pangolin_LIBRARIES}
)

