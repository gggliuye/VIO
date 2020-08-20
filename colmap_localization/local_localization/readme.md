# 1. Test SIFT

## 1.1 Prepare the test data.

Using the new version of my map (made by LiuYe), which is made using the image captured from our new scan application.
Using tools_map for process.

 [MAP INFO] sparse feature map 
            camera number : 1
            point number : 174385
            image number : 1258

* Saved localization binary map.

./Save_map /home/viki/Lucas/garden/garden_v2/database.db /home/viki/Lucas/garden/garden_v2/sparse/ /home/viki/Lucas/garden/garden_v2/SavedMap.dat /home/viki/Lucas/garden/garden_v2/keyframes.txt

Result in a 236Mb map file, with 1258 keyframes inside.

* Image frames with pose localized using the BoW method.

./Test_video_images /home/viki/Lucas/garden/garden_v2/database.db /home/viki/Lucas/garden/garden_v2/sparse/ /home/viki/Lucas/garden/garden_v2/VocIndex.bin /home/viki/UTOPA/Server_Localization/Maps/winter_garden_test/ 596.1

==> Success rate 0.546154 [ 71 / 130 ]
==> Average time,  success: 0.318083, fail : 0.388475

## 1.2 Run the localization process

In this project.

./Test_video_images /home/viki/Lucas/garden/garden_v2/SavedMap.dat /home/viki/UTOPA/Server_Localization/Maps/winter_garden_test/ /home/viki/Lucas/garden/garden_v2/sparse/video_result.txt 596.1 1


## 1.3 Test the match method of Colmap 

Which may be more robust.

* MatchSiftFeaturesCPU/GPU : FLANN method (two way cross check)
* EstimateAbsolutePose() : LORANSAC<P3PEstimator, EPNPEstimator> "Locally Optimized RANSAC" Ondrej Chum, Jiri Matas, Josef Kittler, DAGM 2003.
* RefineAbsolutePose() : ceres (ceres::DENSE_QR), and using CauchyLoss, also offer the option to refine focus length.

My implementation (adding ceres for optimization):

* Candidate keyframes selection based on shared view.
* Two-way FLANN based feature matching.
* P3P-RANSAC based pose estimation.
* Ceres based pose refinement.

# 2. Test ORB

## 2.1 Make orb feature Map - Feature match

./MakeOrbMap /home/viki/Lucas/garden/garden_v2/keyframes.txt /home/viki/Lucas/garden/garden_v2/CameraParameter.txt /home/viki/Lucas/garden/garden_v2/images/


* Test 1. FLANN match sequential keyframes. -> poor performance (too few matches, and almost no plants' point is matched).
* Test 2 : Optical Flow Track. -> Satisfying results.
* Test 3 : use initial depth guess then apply local region search. -> TODO


