cmake_minimum_required(VERSION 2.8.3)
project(vins_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)


message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}

  #${PROJECT_SOURCE_DIR}/LoopClosure/fbow/src/
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_library(camera_model SHARED
    src/camera_models/chessboard/Chessboard.cc
    src/camera_models/calib/CameraCalibration.cc
    src/camera_models/camera_models/Camera.cc
    src/camera_models/camera_models/CameraFactory.cc
    src/camera_models/camera_models/CostFunctionFactory.cc
    src/camera_models/camera_models/PinholeCamera.cc
    src/camera_models/camera_models/CataCamera.cc
    src/camera_models/camera_models/EquidistantCamera.cc
    src/camera_models/camera_models/ScaramuzzaCamera.cc
    src/camera_models/sparse_graph/Transform.cc
    src/camera_models/gpl/gpl.cc
    src/camera_models/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})



ADD_LIBRARY(LoopClosure SHARED
    LoopClosure/ReadOfflineMap.cc
    LoopClosure/orbextractor.cpp
    LoopClosure/Converter.cc
    LoopClosure/PnPsolver.cc
)

target_link_libraries(LoopClosure 
  ${OpenCV_LIBS}  
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/LoopClosure/DBoW2/lib/libDBoW2.so
  #${PROJECT_SOURCE_DIR}/LoopClosure/fbow/build/src/libfbow.so
)



ADD_LIBRARY(MyVio SHARED
    src/System.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/feature_tracker.cpp

    src/utility/utility.cpp

    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp

    src/backend/vertex.cc
    src/backend/edge.cc
    src/backend/problem.cc
    src/backend/vertex_pose.cc
    src/backend/edge_reprojection.cc
    src/backend/edge_imu.cc
    src/backend/edge_prior.cpp
    src/backend/edge_mapfusion.cc
    src/backend/loss_function.cc
    src/backend/imu_integration.cc

)
target_link_libraries(MyVio  
  ${OpenCV_LIBS}  
  ${CERES_LIBRARIES}
  ${Pangolin_LIBRARIES}
  camera_model
  LoopClosure
)

add_executable(run_euroc test/run_euroc.cpp)
target_link_libraries(run_euroc 
  MyVio  
  -lpthread) 

add_executable(run_mineset test/run_mineset.cpp)
target_link_libraries(run_mineset 
  MyVio  
  -lpthread) 

add_executable(run_zju test/run_zju.cpp)
target_link_libraries(run_zju 
  MyVio  
  -lpthread) 

add_executable(run_loop test/loopTest.cpp)
target_link_libraries(run_loop
  MyVio  
  -lpthread) 


#add_executable(run_sim test/run_sim.cpp)
#target_link_libraries(run_sim 
#  MyVio  
#  -lpthread) 


#add_executable(testCurveFitting test/CurveFitting.cpp)
#target_link_libraries(testCurveFitting MyVio)


#add_executable(PointcloudMerge test/PointcloudMerge.cpp)
#target_link_libraries(PointcloudMerge MyVio)

