Dense Reconstruction
===============================

As shown in the Chapter of "Super Panorama", we get a model for total three floors of the "winter garden" scene
with about 1,000,000 faces. with reasonable quality.

We want our image reconstruction to produce a model with similar quality.
We offers two test datasets : `Small indoor scene <https://pan.baidu.com/s/1B3Ar_lXJjYyUNtLQro1NSg>`_ with code "cuxz".
and `Indoor Garden scene <https://pan.baidu.com/s/1aLhItQQ4DRrwEe-cITI9cQ>`_ with code 4em9.

The main problem of the original Colmap dense reconstruction results are :

* Reflection of the smooth floor or windows. ==> Deep learning image segmentaion of the floor, followed by a depth could post-processing step.
* Great amount of noise of texture areas, and very unsatisfying edges. ==> Total Variation resonstruction to preserve sharp edges.
* We also tested a few deep learning reconstruction methods, while they are not satisfying.

1. Colmap MVS
------------------------

* Using the colmap MVS results (using Patch Match algorithm).
* Modeling with poisson reconstruction.
* Simplify the model using Meshlab Quadric Edge Collapse Decimation.

We have a quiet satisfying result in our garden scene (one layer garden part, built with about 1200 images).
It has about 11,000,000 faces before simplification, and 200,000 after simplification.

.. image:: resonstructions/colmap_poisson.png
  :align: center
  :width: 80%

Problems :

* Still too much faces need to be about O(100,000) faces.
* The depth estimation is not complete, result in holes. **Try TV reconstruction**
* The reflection of the ground, and some textureless areas, will lead to poor reconstruction. **Using Deep Learning image segmentation**


2. Deep Learning
----------------------

* Deep learning MVS method.
* `Depth Completion <https://paperswithcode.com/task/depth-completion>`_

The Deep Learning methods are just not stable enough. And training in every datasets is too expensive.

2.1 DeepMVS
~~~~~~~~~~~~~~~~~~~~

We tried DeepMVS in our scene.

.. image:: resonstructions/test_deepmvs.png
  :align: center

Problems:

* It only capture the relative relationship, not the real distance. (see more in `my report <https://gitee.com/gggliuye/VIO/tree/master/DeepMVS>`_ )
* It can only have good result in some scene, while cannot be applied to general cases. It greatly limit its application, as it costs a lot to train in a new scene (main the cost to make the dataset).

2.2 NetMVS
~~~~~~~~~~~~~~~~~~~~~~~~~

Problem:

* The offical NetMVS shows great results, while we found its test data is far too simple. We test it in our own scene, it produces a terrible result. (see `my report jupyter notebook <https://gitee.com/gggliuye/VIO/blob/master/MVSNet/MVSNet_Test.ipynb>`_ )
* The algorithm (we use a `pytorch implementation version <https://github.com/xy-guo/MVSNet_pytorch>`_ ) costs too much GPU memory. Its officical results are built with D=256 (see the explanation of the parameter from the project), while in our 8G GTX1080 GPU, we could only add 10 source images, with D set to 80. Which may explain the poor result.

2.3 CSPN
~~~~~~~~~~~~~~~~~~~~~~~~

We test the pretrained model of `CSPN <https://openaccess.thecvf.com/content_ECCV_2018/papers/Xinjing_Cheng_Depth_Estimation_via_ECCV_2018_paper.pdf>`_ , `github project <https://github.com/XinJCheng/CSPN/tree/master/cspn_pytorch>`_ .
Our results could be seen `here <https://gitee.com/gggliuye/VIO/blob/master/Depth%20Completion/Depth_completion_cspn.ipynb>`_ .
The result is just not satisfying.

.. image:: resonstructions/test_cspn.png
  :align: center

2.4 Sparse-to-Dense
~~~~~~~~~~~~~~~~~~~~~~~~
We test the pretrained model of `sparse-to-dense <https://arxiv.org/pdf/1709.07492.pdf>`_ , `github project <https://github.com/fangchangma/sparse-to-dense.pytorch>`_ .
Our results could be seen `here <https://gitee.com/ggglSummaryWriteriuye/VIO/blob/master/Depth%20Completion/Depth_completion_sparse_to_dense.ipynb>`_ .
The result is just not satisfying.

.. image:: resonstructions/test_sparse_to_dense.png
  :align: center
  :width: 80%

2.5 DeMoN
~~~~~~~~~~~~~~~~~~

`DeMoN: Depth and Motion Network <https://github.com/lmb-freiburg/demon>`_

2.6 Video
~~~~~~~~~~~~~~~~~~~~~

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="//player.bilibili.com/player.html?aid=457051159&bvid=BV125411b7Ww&cid=235161102&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"> </iframe>
    </div>

2.7 Hybrid-method
~~~~~~~~~~~~~~~~~~~~~

`Consistent Video Depth Estimation <https://roxanneluo.github.io/Consistent-Video-Depth-Estimation/>`_ use a various deeplearning method to achieve relative good result.

* Step 1. Colmap : sparse reconstruction.
* Step 2. FlowNet2 : for estimating a flow-displacement consistence evulation.
* Step 3. Fine-Tune (Training) Monodepth2 : use the optical flow displacement consistence (obtained in Step 2.) and geometry consistence (using 3D SFM pyhsics model, obtained in Step 1).
* Step 4. Scale calibration of the monodepth2 depth and colmap depth.



3. Our process
------------------------

Step 1. Semantic segmentation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We using the `Pytorch Encoding Library <https://hangzhang.org/PyTorch-Encoding/model_zoo/segmentation.html>`_ .
And it offers image segmentation models for two datasets (ADE20K Dataset: for indoor scene, and Pascal Context Dataset for outdoor scene).
We use the best result in its dashtable: resnet+deeplab models. And we found the ADE20K Dataset pretrained models are very suitable for our task.
Our result for `Indoor Garden Scene <https://pan.baidu.com/s/1Snslv7AQj24abJQzYxFaUA>`_ with code ipju.

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="//player.bilibili.com/player.html?aid=669503256&bvid=BV1Ha4y1E7Ac&cid=233909622&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"> </iframe>
    </div>

Step 2. Floor repair
~~~~~~~~~~~~~~~~~~~~~~~~~

The floor repair process :

* Extraction the floor point cloud using the semantic segmentation results.
* RANSAC Plane estimation based on these clouds.
* Filter the points far from the plane.
* Filling the area with the estimated plane model.

.. image:: resonstructions/floor_repair.png
  :align: center

Step 3. TV Reconstruction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To try to **fill the depth estimation** .
We try to apply the Total Variation L2 reconstruction (using ADMM algorithm, see more detail in `my convex optimization document <https://cvx-learning.readthedocs.io/en/latest/>`_ )
to refine the depth result of Colmap patch match MVS. (see the example show in `jupyter notebook <https://github.com/gggliuye/SuperPanoama/blob/master/PanoMapping/Mapping_test.ipynb>`_ )

.. image:: resonstructions/tv_test.jpg
  :align: center
  :width: 60%

Problems:

* Too slow. **Use other faster algorithms.**
* Still need refinement. Tried using Deep Learning image segmentation labels (higher TV weight for pixels with the same label), but didn't end up well `example <https://github.com/gggliuye/VIO/blob/master/docs/ServerLocalization/resonstructions/tv_191_use_label.jpg>`_ .

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="//player.bilibili.com/player.html?aid=287072892&bvid=BV1of4y1S7Bx&cid=235161821&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"> </iframe>
    </div>

Step 4. TSDF Reconstruction
~~~~~~~~~~~~~~~~~~~~~~~~~~~

We use a TSDF reconstruction to make our mesh model.


Step 5. Post-process
~~~~~~~~~~~~~~~~~~~~~~~~~~

Post process on the TSDF mesh result.

* remove isolated pieces (wrt Face number) : 25
* cut the undesired parts
* Simplification: Quadric edge collapse decimation : 0.1 reduction, planar simplification

Finally we got a model with 118,403 faces. The results could be found in `Baidu Yun 28gp <https://pan.baidu.com/s/13GZHFTyHT2gI5WsJttjgrg>`_ .

.. image:: resonstructions/show.png
  :align: center
