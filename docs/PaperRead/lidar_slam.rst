Lidar Mapping
====================

2021
----------------------
|thumbs| `MULLS: Versatile LiDAR SLAM via Multi-metric Linear Least Square <https://github.com/YuePanEdward/MULLS>`_.

* Lidar only slam (it doesn't addition information (e.g. imu) for undistortion.)
* Use many kind of features (ground, facade, pillar, beam, etc) to make the system versatile. (and doesn't depends on additional info for cloud, e.g ring).
* Registration to a local submap. solve the new ICP problem based on TEASER (applying truncated least square estimation with semi-definite relaxation).
* Process a loop closure and a pose graph optimization.

|chrown0| `BALM: Bundle adjustment for lidar mapping <https://github.com/hku-mars/BALM>`_ . But the main results shown for Livox Device (denser, small FOV).

* Adaptive voxelization to match both plane and edge features without a segmentation.
* Reduce to pose only BA.

|chrown0| `LiTAMIN2: Ultra Light LiDAR-based SLAM using Geometric Approximation applied with KL-Divergence <https://arxiv.org/abs/2103.00784>`_.
Using a covariance based ICP error, combined with a covariance shape error term (from KL divergence), which allow matching with very large voxel size, then making the registration extremely fast.

2020
-----------------
|chrown0| `LIO-SAM <https://github.com/TixiaoShan/LIO-SAM>`_ Tightly-coupled Lidar Inertial Odometry via
Smoothing and Mapping. In short, add imu pre-integration and sliding window to LOAM.

|thumbs| `OverlapNet: Loop Closing for LiDAR-based SLAM <http://www.roboticsproceedings.org/rss16/p009.pdf>`_.
Top-down 2d view of lidar scan (with other info) for predict overlap rate and yaw.

|unhappy| `ISC (Intensity Scan Context) <https://arxiv.org/abs/2003.05656>`_ Coding Intensity and Geometry Relations for Loop Closure Detection. Encode lidar frame using geometry and intensity info (project intensity into ring distributed subspaces). 

2018
-----------

|chrown0| `LeGO-LOAM <https://github.com/RobustFieldAutonomyLab/LeGO-LOAM>`_ .
Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain.
See `details and some comparisons <https://vio.readthedocs.io/zh_CN/latest/Other/lidarSLAM.html>`_.

2017
----------

|unhappy| `On the performance of metrics to predict quality in point cloud representations <https://core.ac.uk/download/pdf/148032116.pdf>`_.
Using absolute category rating (ACR) and able to perceive distortions.

|unhappy| `On Subjective and Objective Quality Evaluation of Point Cloud Geometry <https://ieeexplore.ieee.org/document/7965681>`_.
Point cloud quality metric using DSIS (double-stimulus impairement scale) methodology. Showing that current state-of-the-art objective 
metrics (point-to-point, point-to-plane or point-to-mesh) do not predict well visual quality, especially under typical distortions such as compression.

|thumbs| `2D SLAM Quality Evaluation Methods <https://arxiv.org/pdf/1708.02354.pdf>`_.
The proportion of occupied and free cells (check wall blurry). The amount of corners and enclosed areas in a map.

2012
--------------
|chrown| `OctoMap <http://www.arminhornung.de/Research/pub/hornung13auro.pdf>`_ `github project <http://octomap.github.io/>`_.
Probabilistic representation, Modeling of unmapped areas, Efficiency (octree).

2009
--------------
|thumbs| `On Measuring the Accuracy of SLAM Algorithms <http://www2.informatik.uni-freiburg.de/~stachnis/pdf/kuemmerle09auro.pdf>`_.
(RPE vs APE) A metric that operates only on relative geometric relations between poses along the trajectory of the robot.


.. |chrown| image:: images/chrown.png
    :width: 3%

.. |chrown0| image:: images/chrown0.png
    :width: 3%

.. |thumbs| image:: images/thumbs.png
    :width: 3%

.. |unhappy| image:: images/unhappy.png
    :width: 3%

.. |question| image:: images/question.png
    :width: 3%
