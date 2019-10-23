Front End : Odometry
========================
 
 As Jakob Engel summarized in [1]_ , SLAM front end can be generally devided into four categories by its methods and representations: 
 
 * **Sparse + Indirect:** This is the most widly used formulation. Estimate the 3d geometry with indirect method, intermediate representation the image with feature points and descriptors, with geometry error of these geometry points. Such as : monoSLAM, PTAM, ORBSLAM, VINS, etc.
 
 * **Dense + Indirect:** This formulation estimates 3D geometry from - or in conjunction with - a dense, regularized optical flow field, thereby combining a geometric error(from the flow field) with a geometry prior(smoothness of the flow field).
 
 * **Dense + Direct:** This formulation employs a photometric error as well as a germetric priot to estimate dense or semi-dense geometry. Examples include DTAM, LSD-SLAM etc.
 
 * **Sparse + Direct:** Thie formulation optimizes a photometric error defined directly on the image, without incorporating a geometric prior. Examples include SVO(optimization based), the work if Jin et al(EKF based).




Feature Extraction And Match
-----------------------------

For the feature, we have various way to detect features. 

* SIFT (mostly used in SFM system), has better property, however it is computationally expensive.
* FAST feature (mostly used in SLAM system), is extremely fast to calculate, however it lacks some important property. 
* And some Deep learning algorithms (e.g. GcnV2 features).

And most feature descriptor algorithms are very expensive to calculate. Even the BRIEF binary descriptor(used as ORB) is lumbersome for moblie phone. We need to carefully weight when to calculate descriptors, when we should use other algorithms(e.g. Optical Flow).


Direct Methods
------------------
we can also use direct methods for tracking. 
Direct (and semi-direct) methods eliminate the need of costly feature extraction and robust matching techniques for motion esimation.
As these algorithms operate directly on pixel intensities, which can result in subpixel precision at high frame-rate.
And as a result of these patch match approaches, they are very robust to fast move.



SVO (Fast Semi-Direct Monocular Visual Odometry)
~~~~~~~~~~~~~~~~~~
SVO [1]_ is designed for **Drones(Micro Aerial Vehicles)** , a semmi-direct method. While the camera is localized under the drone, pointing downside. It can run very fast, 55 FPS on the onboard embedded computer. 

However it is not robust for rotation. When test with AR situations, where exist a lot of rotation vertical, SVO system can easily failed. 

DSO(Direct sparse odometry)
~~~~~~~~~~~~~~~~~~~~~~

`project page <https://vision.in.tum.de/research/vslam/dso?redirect=1>`_


Pose Calculation
---------------------
* Use Homography matrix.
* Use Pnp(in some tracking algorithms) to calculate from 2d-3d points matches.
* Use bundle adjustment to optimization pose(as in ORBSLAM).
* Use Filter methods to calculate.


Match With Map
------------------
We can get a camera pose at this moment, however it is not satisifying at most time.
In most SLAM system(as I know for ORBSLAM and PTAM), the matching processing is adjusted to match between the current frame map points (not with a single frame).

All the map points will be projected into the image frame(by last calculated camera pose). And for each projected map point that is in the current view, best matched point will be found within a radius range around it from the current image frame, this pair of points will be treated as a match.

 * for ORBSLAM, their BRIEF descriptors will be used as criterion.
 * for PTAM, zero mean SSD and cross-correspondence will be used as criterion.


Multi-sensor Fusion
--------------------------

* IMU: 
  IMU is used in a lot of SLAM system, as VINS for drones, and MSCKF for AR kit, etc.

* GPS / Blue Tooth : They can difinitly give us some information. The problem is how to use it.

* LIDAR: e.g.VLOAM


Our VIO front-end
------------------------

Feature Extraction
~~~~~~~~~~~~~~~~~~~
Basicly, will use good feature to track (Shi-Tomasi method) with mask.

    cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);

* Make mask for detection (version 1):
    1. Make a circle mask for fish eye camera.
    2. Sort all the points by its tracked count.
    3. Make mask based on its tracked count's rank with a circle.
    4. If a point is already in the mask, skip the point. 
    5. As a result, the feature extraction will favor the long tracked points.

* Make mask for detection (version 2), I found the rank process may not be that profitable:
    1. Make a circle mask for fish eye camera.
    3. Make mask around all tracked points with a circle.

* Update the state. Save the current data to the last data.


Optical flow
~~~~~~~~~~~~~~~~~~~
Basicly, we used pyramid LK optical flow to track features. 

    cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

Then we will filter the tracked points :

* If the input point is in the image border. As for optical flow, the points can be followed into border, where the points will be kept and effect greatly the tracking result.
* Reject point by fundamental check. This process will be done with the following steps:
    1. Lifts a point from the image plane to its projective ray.
    2. project the point back to the image plane with another focus length, obatin un_pts.
    3. use the un_cur_pts and un_forw_pts to calculate fundamental matrix with RANSAC.
    4. As a result, the plane points will be reduced and line points will be kept.
    5. Experiments show this process optimized the robustness, and had spent almost no time.



Reference
-------------------------------

.. [1] Forster C, Pizzoli M, Scaramuzza D. SVO: Fast semi-direct monocular visual odometry[C]//2014 IEEE international conference on robotics and automation (ICRA). IEEE, 2014: 15-22.

.. [2] Engel J, Koltun V, Cremers D. Direct sparse odometry[J]. IEEE transactions on pattern analysis and machine intelligence, 2017, 40(3): 611-625.
