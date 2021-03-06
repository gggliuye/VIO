Super Panorama
=======================

**Single Image Localization** based on Deeplearning, and a lidar scan prebuilt map.

* Calibration of the panorama images. Analysis of the panorama camera model.
* Faro Scan.
* Classic method test (BOW + SIFT + FLANN + PNP-RANSAC + optimization).
* Deep learning method test (NetVLAD + SuperPoint + SuperGlue + PNP-RANSAC + optimization).

1. Panorama Image
------------------

Use the panorama image to estimate.

* Localize the panorama images in our prebuilt map.
* ICP to estimate relative transformation.


1.1 Panorama camera model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Generate a cube point cloud for simulation

.. image:: images/cube.PNG
  :align: center
  :width: 30%

Generate the panorama image using the `scripts <https://github.com/gggliuye/VIO/blob/master/pretreatment/Panorama_Distort.ipynb>`_

.. image:: images/panorama_1.jpg
  :align: center
  :width: 60%

There exist variations on the sphere model center. While we can always obtain a fine result.
Using some simple method we can transform the panorama image into a pinhole camera image (for an example `python <https://github.com/adynathos/panorama_to_pinhole>`_ ).

1.2 Faro Scan
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We also tried to use the raw image from faro, unfortunately it is of bad quality.

* It is captured by a spinning device -> lack of uniform exterior calibration.
* About 40% of the image is consist of the outer shell of the Lidar Device.
* It is registered by the lidar scan data -> hard for us to calibration.

We turn to the unified panorama image output of Faro. We found the lower part is hidden.
So we complete the missing part, and apply the camera model to the lidar scan, result in an
accurate RGB image with aligned depth image.

Using the `code <https://github.com/gggliuye/VIO/blob/master/panorama_images/panorama_extraction_perfect_sphere.ipynb>`_
We can get the aligned depth image :

.. image:: images/pano_faro.PNG
  :align: center

And some sample of pinhole camera images:

.. image:: images/pinhole_faro.PNG
  :align: center

**Calibration of the Faro Device**. we found the output panorama images of Faro has constant height H (which miss a part in the bottom),
and different width W (which make the panorama images to have observable boundary).
I introduce a margin variable M to fix the height, and use twice the length of the height to assign width.
Finally I resize the panorama images in to shape :math:`(H+M)\times 2(H+M)` , Which makes a perfect calibration for our later tests.

**Problems** of the images : We have two main problems **The intersection of the panorama images** and **Noise in the depth data**.

* As the un-uniformed panorama images of the Faro device, as we descussed above, we sometimes have to leave a black margin in the projected pinhole image. which could be seen in the left image below (as a black line obervable in the left half).
* As we didn't exclude all the moving people, nor other noise object. We could end up with lots of bruits in the lidar scan data (as shown in the right image below, there are lots of shadows in the depth image). Which could affect the localization process, and also the 3d model reconstruction process.

.. image:: images/problematic_images.png
   :align: center
   :width: 80%

**Reconstruction** using TSDF. The mesh (with scale 0.1) could be found in `Baidu Drive with code arot <https://pan.baidu.com/s/1OSKP5dQl62NMPHtp_x7rTQ>`_ ,
or in `Google Drive <https://drive.google.com/file/d/11LVCc8Yi5HtLM5OBz-wjPoneXxJ7ZAlO/view?usp=sharing>`_ .

.. image:: images/mesh_2.png
  :align: center
  :width: 80%


2. Localization using SIFT
------------------------------

This is for test the possiblity of using a simple SIFT for a large scene localization.
For a validation of this thought, we only apply one panorama image and some reasonable query images for test.
The objective here is to test this method, and to offer a base.

* Firstly, create several keyframe images (in pinhole camera model) and its corresponding aligned depth, using the 3D Lidar Scan data.

.. image:: images/transformed_depth.PNG
  :align: center

* Then, realize a simple SIFT (using RootSIFT) feature matching system with PnP pose estimation process.
The results are shown below:

.. image:: images/match_res.PNG
  :align: center

* The left image is the query image, the middle image is the matched keyframe, the right image is the rendered image using the panorama scan and the estimated pose.

From the upper test, we found the following problems:

* SIFT could only match a limited number of features (some well structed points), while it cannot handle some hard cases (for the plants).
* With the limited matches, the pose estimation is far from ideal.
* We need a better feature extraciton and matching strategy, for an example using SuperPoint + SuperGlue.


3. Localization using DL
------------------------------
Here we test the pipeline of Deep learning.

* **Make the dataset** : we make a dataset of the indoor complicated scene, with 133 lidar scans. Which includes indoor plants scene, indoor market, and some outdoor views. Our test query images were taken at least 2 weeks before.
* **Pretreatment** : for matching with query image, we project the panorama images to several pinhole model images, as is shown in the chapters before.
* **NetVLAD Index** : here we extract the global descriptor for the database images using a VGG-encoded NetVLAD network. And using SQLite3 for save the results.
* **Feature extraction** : we use SuperPoint (pretrained model) as our feature extraction. And using SQLite3 for save the results. (result in a 1.2G database)
* **Feature matcher** : we use SuperGlue (indoor pretrained model) as our feature matcher.
* **Pose Estimation** : we use a EPnP-RANSAC method for pose estimation.
* **Pose Refinement** : we use a iterative optimization method for pose refinement.

3.1 Image Retrival
~~~~~~~~~~~~~~~~~~~~~~~~~

**NetVLAD pretrained** We tested the pretrained NetVALD model (with an vgg front-end).
We found that the the pretrained model performs badly in our dataset. while require us to train on our data.

**BOW** : To compare the performance of the retrained NetVLAD result, we use a BOW model as reference. While use the SuperPoint descripotrs,
and uses a 1000 words vocabulary.
And in our later tests, we use the BOW model to maintain a stable version of the algorithm.

3.2 Features
~~~~~~~~~~~~~~~~~~

We use the pretrained SuperPoint and SuperGlue, and they do offer a great result.

3.3 Pose
~~~~~~~~~~~~~~~~~~~

* We use a P3P-RANSAC based pose estimation algorithm for a fast pose estimation, while will also achieve a stable outlier rejection.
* Then we apply a Optimization based method for the refinement of the pose. In our first tests, we use the iterative method offered by OpenCV, which gave a very unstable result. We found that a normal Newton iterative method is not robust enough, as we could still includes a few outliers. So I developped a **Ceres based Optimization based method with robust loss function** for the pose refinement task, which gives a very robust result.

3.4 Results
~~~~~~~~~~~~~~~~~

We got ideal results. The follwoing image shows the result for the same query image, as the former chapter.
See more result images in `Baidu Driver with coe rqmy <https://pan.baidu.com/s/1icp7K-1BXvT_ykWb9-d1Rg>`_ .

.. image:: images/superglue.jpg
  :align: center

Where the first image is the query image, the second image is the matched reference keyframe proposed by NetVLAD. SuperGlue matches are shown
in the images. Using these matches we got our pose estimation T. Using T we project the whole panorama image into the virtual camera plane,
with the depth image shown in the third column. To compare the error, we extract the edges in the depth image, then praint them into our query image,
which results in the fourth column.

**Run time** :

Here we show the run-time histograms in our tests (using i7 CPU and RTX2080 GPU) for each candidate keyframe.

.. image:: images/run_time.png
  :align: center
  :width: 80%

In real application, we will process for multiply keyframes for a single input query image.
It requires a well designed keyframe proposition algorithm, to most drasticly reduce the calculation time.

**Succeed Cases** :

.. image:: images/sg_succeed.png
  :align: center

We could observe that there exists nosie both in the keyframe images and the depth data (both result from the
moving objects shown in the view). Generally, our pipeline
could offer a quite satisfying result. While when there is a vast view point changement (the last row),
the pose estimation may be less accurate.

**Unity Demo** : Here we show our demo, to combine our localization system with a local SLAM (we used ARCore) to realize a large scene consist AR application.

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="//player.bilibili.com/player.html?aid=626953712&bvid=BV1et4y1S778&cid=229955696&page=2" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"> </iframe>
    </div>

3.5 Advantages
~~~~~~~~~~~~~~~~~~~~~~~

* Much more robust to lightness changes than traditional methods.
* Much more robust to view-point changes than traditional methods.
* Much more accurate.
* A great mesh model could be offered.
* Can match most of the plants points (after a lone period), which is impossible for traditional methods.

3.6 Problems
~~~~~~~~~~~~~~~~~~~~~~

* Shared scene of the first floor and the second floor. Lead to negative match.

.. image:: images/1245.jpg
  :align: center

* Drasticly changed scene. (lead to large error, while we could still match successfully)

.. image:: images/1255.jpg
  :align: center

* Too few distinctable features within the plants. Lead to negative match.

.. image:: images/1735.jpg
  :align: center

.. image:: images/1805.jpg
  :align: center

4. TODOs
------------------------

* **Dataset** : there are problems with data, as seen in chapter 2. we should deal with it.
* **NetVLAD** : the performance of the pretrained NetVLAD is not ideal, better to train in our dataset.
* **Parameters tune** : we could further choose better parameter threshold for the feature match phase.
* **Pose Refinement** : I am consider to use a l1-norm based optimization method for the pose refinement (e.g. use ADMM method).
