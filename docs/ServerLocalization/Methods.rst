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

**Feature** : use multi-scale dense CNN features. This paper uses VGG-16 for global descriptor, and use DenseSIFT extractor and its RootSIFT
descriptors from `VLFeat <https://www.vlfeat.org/overview/dsift.html>`_ for local features. **DenseSIFT** The main advantage of
using vl_dsift over vl_sift is speed, and the obvious difference is that with dense SIFT you get a SIFT descriptor
at every location, while with normal sift you get a SIFT descriptions at the locations determined by Lowe's
algorithm. **RootSIFT**: L1 normalize the SIFT vector, then take square root of each element (for more, see
`Arandjelovic and Zisserman, 2012 paper <https://www.robots.ox.ac.uk/~vgg/publications/2012/Arandjelovic12/arandjelovic12.pdf>`_ paper [1]_).
Then, compare RootSIFT descriptors using Euclidean distance is equivalent to using the Hellinger kernel to compare the orignal SIFT vectors
(it is benefit to use Hellinger kernel to compare histogram distance) .


**Image Retrive** : NetVLAD. Will evaluate the top-10 matched images.

**Pose Estimation** : use RANSAC-based dense feature match, dense pose estimation (DensePE), and dense pose validation
(DensePV). These 'Dense' processes are mainly match the query image with the synthesized view using 3D scans.

**Result** rate (%) of correctly localized querier within given distance(m) threshold and within 10 degree angular error:

+--------+--------------+--------------+------------------+--------------+
|        |  Direct2D-3D |  BoW-SparsePE| NetVLAD+SparsePE |        InLoc |
+========+==============+==============+==================+==============+
| 0.25m  |  11.9        |  20.1        |  21.3            |  38.9        |
+--------+--------------+--------------+------------------+--------------+
| 0.5m   |  15.8        |  29.5        |  30.7            |  56.5        |
+--------+--------------+--------------+------------------+--------------+
| 1.0m   |  22.5        |  41.0        |  42.6            |  69.9        |
+--------+--------------+--------------+------------------+--------------+

.. [1] From the paper *Three things everyone should know to improve object retrieval*. This paper contributs three parts: **RootSIFT** (as descripted above). Secondly, **DQE** (Discriminative query expansion) which is a linear SVM discriminative for image retrival. And finally **AUG** (Database-side feature augmentation).

Another paper is also mentioned to be closly related to its DensePE process `Fixing the Locally Optimized RANSAC <https://www.researchgate.net/publication/259338571_Fixing_the_locally_optimized_RANSAC>`_ , which proposed LO+-RANSAC.
