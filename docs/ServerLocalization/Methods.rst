Related Works
==========================

This page shows some works of single image based localization task.

1. InLoc
----------------------

`InLoc: Indoor Visual Localization with Dense Matching and View Synthesis <https://arxiv.org/abs/1803.10368>`_

* Need dense map (lidar devices scanned map) for match query single images.
* Have open source Maplab code.
* Basic prcess : Deep learning feature extraction -> NetVLAD -> DensePE -> DensePV.
* Provides database for offical CVPR evaluation.

**Database** : Indoor scene (Washington University) with panoramic 3D scans at 177 distinct positions. 329 test query images (took by iphone7).

**Map** : built with lidar 3D scans, of high accuracy, with depth map provided.

**Feature** : use multi-scale dense CNN features. (use VGG-16 for global descriptor, and use DenseSIFT extractor and its RootSIFT descriptors from VLFearfor local features )

**Image Retrive** : NetVLAD. Will evaluate the top-10 matched images.

**Pose Estimation** : use RANSAC-based dense feature match, dense pose estimation (DensePE), and dense pose validation (DensePV).

**Result** rate (%) of correctly localized querier within given distance(m) threshold and within 10 degree angular error:

+--------+--------------+--------------+------------------+--------------+
|        |  Direct2d-3D |  BoW-SparsePE| NetVLAD+SparsePE |        InLoc |
+========+==============+==============+==================+==============+
| 0.25m  |  11.9        |  20.1        |  21.3            |  38.9        |
+--------+--------------+--------------+------------------+--------------+
| 0.5m   |  15.8        |  29.5        |  30.7            |  56.5        |
+--------+--------------+--------------+------------------+--------------+
| 1.0m   |  22.5        |  41.0        |  42.6            |  69.9        |
+--------+--------------+--------------+------------------+--------------+
