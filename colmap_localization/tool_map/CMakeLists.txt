cmake_minimum_required(VERSION 3.10)
project(ulocalization)

set(FOLDER_NAME "tools")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

message( ${JNI_INCLUDE_DIRS})

message( ${JNI_LIBRARIES})

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/JNIs/
${PROJECT_SOURCE_DIR}/src/
${PROJECT_SOURCE_DIR}/visual_index/

#${JSONCPP_INCLUDE_DIRS}
#/usr/lib/jvm/java-8-openjdk-amd64/include
#/usr/lib/jvm/java-8-openjdk-amd64/include/linux
${JNI_INCLUDE_DIRS}
${COLMAP_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}

)

link_directories(${COLMAP_LINK_DIRS})

add_library(ulocalization SHARED

src/test_utils.cc
src/UlocalizationPlugins.cc
src/UlocalizationPluginsBeta.cc
src/LocalizationLY.cc
src/configuration.cc
src/Viewer.cc
src/MapSaver.cc
src/MarkerFinder.cc

JNIs/Java_com_utopa_arserver_localization_jni_LocalizationSDK.cpp
)

target_link_libraries(ulocalization
  ${OpenCV_LIBS} 
  ${JNI_LIBRARIES}
  ${COLMAP_LIBRARIES}
  ${Pangolin_LIBRARIES}
#/usr/lib/jvm/java-8-openjdk-amd64/jre/lib/amd64/libjawt.so
#/usr/lib/jvm/java-8-openjdk-amd64/jre/lib/amd64/server/libjvm.so
)


add_executable(example tests/example.cc)
target_link_libraries(example 
  ulocalization
)

add_executable(Test2 tests/Test2.cc)
target_link_libraries(Test2 
  ulocalization
)


add_executable(Test_video_images tests/Test_video_images.cc)
target_link_libraries(Test_video_images 
  ulocalization
)

add_executable(Test_markers tests/Test_markers.cc)
target_link_libraries(Test_markers 
  ulocalization
)

add_executable(Test_initpose tests/Test_initpose.cc)
target_link_libraries(Test_initpose 
  ulocalization
)

add_executable(make_index tests/make_index.cc)
target_link_libraries(make_index
  ulocalization
)

add_executable(test_plugin tests/testPlugin.cc)
target_link_libraries(test_plugin
  ulocalization
)

add_executable(Test_arcore tests/Test_arcore.cc)
target_link_libraries(Test_arcore
  ulocalization
)

add_executable(Save_map tests/Save_map.cc)
target_link_libraries(Save_map
  ulocalization
)
