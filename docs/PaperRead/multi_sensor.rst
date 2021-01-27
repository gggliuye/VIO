Multi-Sensor Mapping
==================

2020
---------------

|chrown0|  `CamVox <https://github.com/ISEE-Technology/CamVox>`_, Lidar visual mapping using livox.

* Livox generate dense lidar cloud, match visual edge with lidar intensity image edge for extrinsic parameters calibration.
* IMU for lidar un-distortion.
* Run ORBSLAM2 RGBD pipeline.

|thumbs|  `Augmenting Visual Place Recognition with Structural Cues <http://rpg.ifi.uzh.ch/research_vo.html>`_
Use both image (e.g. `NetVLAD 2016 <https://arxiv.org/abs/1511.07247>`_) and 3d cloud (e.g.
`PointNetVLAD 2018 <https://arxiv.org/abs/1804.03492>`_) encoders.

|unhappy|  `Stereo Localization in LiDAR Maps <https://github.com/tony1098/Stereo-Localization-in-LiDAR-Maps>`_ .
Localize stereo camera in pre-built lidar map.

* Using ZED stereo camera, opencv (StereoSGBM and DisparityWLSFilter) to compute depth image.
* Registration using `Nelder-Mead method <https://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method>`_ .

|unhappy| `RGB2LIDAR: Towards Solving Large-Scale Cross-Modal Visual Localization <https://arxiv.org/abs/2009.05695>`
DL match rgb image and depth image (from lidar cloud)


2019
------------

|unhappy|  `CMRNet: Camera to LiDAR-Map Registration <https://github.com/cattaneod/CMRNet>`_.
Project a depth into plane (from an initial pose guess), CMRNet use RGB and depth as input, output 2D correspondings for each depth value.
Finally PnP-RANSAC for pose estimation.


2018
-----------

2017
---------------

|question| `DSAC Differentiable RANSAC <https://github.com/cvlab-dresden/DSAC>`_. replace non-differentiable parts of
RANSAC algorithm with approximated differentiable parts (by soft argmax and probabilistic selection).
Then make a deep learning DSAC. (As I understand, RANSAC is mathematically proved, I don't understand how its accuracy can be improved).

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
