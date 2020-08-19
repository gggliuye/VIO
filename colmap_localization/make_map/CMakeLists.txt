

cmake_minimum_required(VERSION 3.10)
project(uMapping)

set(FOLDER_NAME "tools")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
#set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --compiler-options -fPIC")

#set(DCUDA_CUDART_LIBRARY  "=/usr/local/cuda/lib64/libcudart.so")

find_package(OpenCV 3.4 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

set(CUDA_MIN_VERSION "7.0")
if(CUDA_ENABLED)
    find_package(CUDA ${CUDA_MIN_VERSION} QUIET)
endif()

find_package(PkgConfig REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Pangolin REQUIRED)

find_package(JNI REQUIRED)
if(NOT JNI_FOUND )
   message(FATAL_ERROR "JNI not found.")
endif()

find_package(COLMAP REQUIRED)

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/JNI
${PROJECT_SOURCE_DIR}/src

#/usr/lib/jvm/java-8-openjdk-amd64/include
#/usr/lib/jvm/java-8-openjdk-amd64/include/linux
${JNI_INCLUDE_DIRS}
${COLMAP_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
)

link_directories(${COLMAP_LINK_DIRS})

#Message(INFO ${COLMAP_LIBRARIES} )

add_library(umapping SHARED
  src/u_mapping.cc
  src/test_utils.cc
  src/UMappingPlugins.cc
  src/LocalizationLY.cc
  src/configuration.cc
  src/Viewer.cc
  src/ScaleEstimator.cc
  src/StateCallbacks.cc

  src/AnalysisTool.cc

  JNI/com_utopa_arserver_makemap_jni_MakeMapSDK.cpp
)


target_link_libraries(umapping
  ${OpenCV_LIBS} 
  ${Pangolin_LIBRARIES}
  ${COLMAP_LIBRARIES}
  ${JNI_LIBRARIES}
  /usr/local/lib/colmap/libcolmap_cuda.a

 #/usr/lib/jvm/java-8-openjdk-amd64/jre/lib/amd64/libjawt.so
 #/usr/lib/jvm/java-8-openjdk-amd64/jre/lib/amd64/server/libjvm.so
)

add_executable(Make_map test/example.cc)
target_link_libraries(Make_map 
  umapping
)


add_executable(Extract_image test/example_extract_image.cc)
target_link_libraries(Extract_image 
  umapping
)

add_executable(Make_map_add test/example_add.cc)
target_link_libraries(Make_map_add
  umapping
)

add_executable(Test_scale test/Test_scale.cc)
target_link_libraries(Test_scale
  umapping
)

add_executable(AnalysisMap test/AnalysisMap.cc)
target_link_libraries(AnalysisMap
  umapping
)
