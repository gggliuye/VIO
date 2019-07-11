Front End : Odometry
========================


Feature Extraction And Match
-----------------------------

For the feature, we have various way to detect features. 

* SIFT (mostly used in SFM system), has better property, however it is computationally expensive.
* FAST feature (mostly used in SLAM system), is extremely fast to calculate, however it lacks some important property. 
* And some Deep learning algorithms (e.g. GcnV2 features).

And most feature descriptor algorithms are very expensive to calculate. Even the BRIEF binary descriptor(used as ORB) is lumbersome for moblie phone. We need to carefully weight when to calculate descriptors, when we should use other algorithms(e.g. Optical Flow).


Direct Methods
------------------
we can also use direct methods for tracking. Depends on the intensity of pixels.
SVO LSD_SLAM

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
