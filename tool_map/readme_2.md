
./make_index /home/viki/UTOPA/Server_Localization/Maps/winter_garden/database.db /home/viki/UTOPA/Server_Localization/Maps/winter_garden/dense/sparse/ /home/viki/UTOPA/Server_Localization/Maps/winter_garden/VocIndex_zf.bin /home/viki/UTOPA/Server_Localization/vocabs/vocab_garden.bin

./Test_video_images /home/viki/UTOPA/Server_Localization/Maps/winter_garden/database.db /home/viki/UTOPA/Server_Localization/Maps/winter_garden/dense/sparse/ /home/viki/UTOPA/Server_Localization/Maps/winter_garden/VocIndex_zf.bin /home/viki/UTOPA/Server_Localization/Maps/winter_garden_test/


./Test_video_images /home/viki/Lucas/garden_resized/database.db /home/viki/Lucas/garden_resized/sparse/ /home/viki/Lucas/garden_resized/VocIndex32.bin /home/viki/UTOPA/Server_Localization/Maps/winter_garden_test/

./Test_video_images /home/viki/Lucas/garden_resized/database.db /home/viki/Lucas/garden_resized/sparse/ /home/viki/Lucas/garden_resized/VocIndex_m65k.bin /home/viki/UTOPA/Server_Localization/Maps/winter_garden_test/


## 2020/06/16

### Tested in real scene

* The accuracy is relatively high, for most of the cases. While, for the part of the scene with some small structured buildings, the mesh didn't match well. 
* The success rate is extremely low. which is the major part of later work. (To find out the reason, and optimize the condition)

## 2020/06/17

* For this scene, we have more than 2000 image frames in the database. However, the images are mostly the same, From logical thought, we should not us a large vocabulary tree for it. So I will test the image localization success rate for both cases.

#### Test VocIndex_32K
* Slow (1~2s)
* ==> Success rate 0.100000 [ 13 / 130 ]

#### Test VocIndex_256K
* match speed about two times faster (~0.7s)
* ==> Success rate 0.161538 [ 21 / 130 ]

#### Test VocIndex_1M
* Faster (~0.5s)
* ==> Success rate 0.123077 [ 16 / 130 ]


### TODO

ZeFan:
* Resize image compare success rate and accuracy.
* Build vocab tree

LiuYe:
* SuperPoint -> 256 float
  SIFT -> 128 uint8

## 2020/06/18 Test small size images

#### Test LY : VocIndex_32K , 256K, 1M
* 1580 images in database
==> Success rate 0.038462 [ 5 / 130 ]
==> Success rate 0.061538 [ 8 / 130 ]
==> Success rate 0.069231 [ 9 / 130 ]


#### Test ZeFan : VocIndex_32K , 256K, 1M
* 2264 images in database
==> Success rate 0.161538 [ 21 / 130 ]
==> Success rate 0.153846 [ 20 / 130 ]
==> Success rate 0.138462 [ 18 / 130 ]

* Zefan built voc tree
time ~0.3s
==> Success rate 0.307692 [ 40 / 130 ]


## 2020/06/22 Recollect images
* 829 images in database : VocIndex_32K, 256K, 1M, Ours
==> Success rate 0.746154 [ 97 / 130 ]
==> Average time,  success: 0.613619, fail : 0.618438

==> Success rate 0.830769 [ 108 / 130 ]
==> Average time,  success: 0.397337, fail : 0.441620

==> Success rate 0.846154 [ 110 / 130 ]
==> Average time,  success: 0.348255, fail : 0.411242

==> Success rate 0.961538 [ 125 / 130 ]
==> Average time,  success: 0.331234, fail : 0.266508


## 2020/06/30 Final of first period

./make_index /home/viki/UTOPA/Server_Localization/Maps/winter_garden_final/database.db /home/viki/UTOPA/Server_Localization/Maps/winter_garden_final/dense/sparse/ /home/viki/UTOPA/Server_Localization/Maps/winter_garden_final/VocIndex_zf.bin /home/viki/UTOPA/Server_Localization/vocabs/vocab_garden.bin

./Test_video_images /home/viki/UTOPA/Server_Localization/Maps/winter_garden_final/database.db /home/viki/UTOPA/Server_Localization/Maps/winter_garden_final/dense/sparse/ /home/viki/UTOPA/Server_Localization/Maps/winter_garden_final/VocIndex_zf.bin /home/viki/UTOPA/Server_Localization/Maps/winter_garden_test/

* 1217 images in the database
==> Success rate 0.969231 [ 126 / 130 ]
==> Average time,  success: 0.316191, fail : 0.312439



