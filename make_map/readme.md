* curl warning

sudo rm -rf /usr/local/lib/libcurl.so.4
sudo ln -s /usr/lib/x86_64-linux-gnu/libcurl.so.4.4.0 /usr/local/lib/libcurl.so.4

## Remark

* The work space should be empty, as I will remove all the files in it. I hope this will not delete some files important.
* all the images in one folder must be taken by the same type of device. (and image folder name should be longer than 4 char)

## Parameters note

* float feature_parallax_threshold : The minimal parallax between two extracted frames. 
     larger  -> less frames
     smaller -> more frames

## Test build

./Make_map work_space_path resource_path

./Make_map /home/viki/UTOPA/Server_Localization/Maps/build_test/work_space_2 /home/viki/UTOPA/Server_Localization/Maps/build_test/build_sources_2 /home/viki/UTOPA/Server_Localization/vocabs/vocab_tree_flickr100K_words32K.bin /home/viki/UTOPA/Server_Localization/vocabs/vocab_tree_flickr100K_words256K.bin /home/viki/UTOPA/Server_Localization/vocabsvocab_tree_flickr100K_words1M.bin 

./Make_map /home/viki/UTOPA/Server_Localization/Maps/Winter_v1/work_space_2 /home/viki/UTOPA/Server_Localization/Maps/Winter_v1/resources /home/viki/UTOPA/Server_Localization/vocabs/vocab_tree_flickr100K_words32K.bin /home/viki/UTOPA/Server_Localization/vocabs/vocab_tree_flickr100K_words256K.bin /home/viki/UTOPA/Server_Localization/vocabsvocab_tree_flickr100K_words1M.bin 

## Test Extract_image

./Extract_image /home/viki/UTOPA/Server_Localization/Maps/winter_garden /home/viki/Lucas/winter_garden 50 20 0.3

./Extract_image /home/viki/UTOPA/Server_Localization/Maps/winter_garden_final /home/viki/Lucas/winter_garden_final 50 20 0.3

## Test add images

./Make_map_add /home/viki/UTOPA/Server_Localization/Maps/build_test/work_space_2 /home/viki/UTOPA/Server_Localization/Maps/build_test/addition_resources /home/viki/UTOPA/Server_Localization/vocabs/vocab_tree_flickr100K_words32K.bin /home/viki/UTOPA/Server_Localization/vocabs/vocab_tree_flickr100K_words256K.bin /home/viki/UTOPA/Server_Localization/vocabsvocab_tree_flickr100K_words1M.bin 

## Test scale calculation

./Test_scale /home/viki/UTOPA/Server_Localization/Test_kexuecheng_B/work_space/database.db /home/viki/UTOPA/Server_Localization/Test_kexuecheng_B/work_space/sparse/ /home/viki/UTOPA/Server_Localization/Test_kexuecheng_B/work_space/VocIndex.bin /home/viki/UTOPA/Server_Localization/Test_kexuecheng_B/ArCore_result/Trajectory.txt /home/viki/UTOPA/Server_Localization/Test_kexuecheng_B/ArCore_result/images/ 496 282

./Test_scale /home/viki/UTOPA/Server_Localization/Maps/Winter_v1/work_space/database.db /home/viki/UTOPA/Server_Localization/Maps/Winter_v1/work_space/sparse/ /home/viki/UTOPA/Server_Localization/Maps/Winter_v1/work_space/VocIndex.bin /home/viki/UTOPA/Server_Localization/Maps/Winter_v1/resources/trajdata/Trajectory.txt /home/viki/UTOPA/Server_Localization/Maps/Winter_v1/resources/trajdata/images/ 496 282

./Test_scale /home/viki/Lucas/0428/winter_line_out_v1/database.db /home/viki/Lucas/0428/winter_line_out_v1/sparse /home/viki/Lucas/0428/winter_line_out_v1/VocIndex.bin /home/viki/Lucas/0428/winter_line_out_v1/Trajectory.txt /home/viki/Lucas/0428/winter_line_out_v1/images/arcore/ 496 200


// 282 images : 0.180249

## State manager 

1. DataPrepare : seperate videos. 
2. Feature Extractor : no built in callback exist. -> but I can roughly estimate the time
3. Feature Matcher : no built in callback exist. -> but I can roughly estimate the time
4. Sparse Reconstruction : has callback well built.
5. Dense Reconstruction : no built in callback exist. -> estimate by counting files in workspace folder
   * photometric 
   * geometric
   * fusion : estimate 


## winter scale

Done


## Further

* Set the video camera id to identity, and optimize the video camera parameters.
* Set the maximal image size of the MVS process to accelerate.
* Loop closure parameters could be modified.

