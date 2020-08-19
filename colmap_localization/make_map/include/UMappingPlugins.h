/**
*    plugins of Ulocalization for Server Use
*
*    LIU YE @ moon light
*
*/


#ifndef ULOCALIZATION_PLUGINS_H
#define ULOCALIZATION_PLUGINS_H

#include "u_mapping.h"

/*
* Set the three vocabulary files for faster matching
* must be in the correct order
* path1 : the smallest (in volume), path2 : the middle one, path3 : the largest vocabulary file
*/
extern "C" int Internal_Set_Vocabulary_Paths(const char *voc_path1, const char *voc_path2, const char *voc_path3);

/*
* Read resources from resource_path, and output result into work_space
* ATTENETION: WE WILL CLEAN THE WORK SPACE BEFORE BUILD
* resource path must be consist of folder videos, and folder images
* quality have four levels 0(low), 1(medium), 2(high), 3(extreme) (default is 0)
* key is the flag id
*/
extern "C" int Internal_Start_Map(const char* work_space, const char* resource_path, bool build_dense, int quality, const char* key_, UMapping::REGISTER_CALLBACK P);

/*
* Read resources from resource_path, and output result into work_space
* The old files in the work space will be kept and used for building map
* resource path must be consist of folder videos, and folder images
* quality have four levels 0(low), 1(medium), 2(high), 3(extreme) (default is 0)
* key is the flag id
*/
extern "C" int Internal_Addition_Images(const char* work_space, const char* resource_path, bool build_dense, int quality, const char* key_, UMapping::REGISTER_CALLBACK P);

/* 
*  void fcn(char* key, int flag)
*  flag = 1  : successfully finished
*  flag = -1 : error occured
*/
extern "C" int Internal_Register_Callback(UMapping::REGISTER_CALLBACK P);


/*
* Get the state of the current building process
* the states are : 
*     MAKEING_IMAGE = 0,
*     FEATURE_EXTRACTING = 1,
*     FEATURE_MATCHING = 2,
*     SPARSE_RECONSTRUCTION = 3,
*     DENSE_RECONSTRUCTION = 4,
*     OUTPUT_PLY_AND_VOC = 5,
*     IDLE = 6,
*     UNKOWN_ERROR = -1,
*/
extern "C" int Internal_Get_Mapping_State();

#endif // ULOCALIZATION_PLUGINS_H
