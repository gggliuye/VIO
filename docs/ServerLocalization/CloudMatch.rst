Panorama Match
=======================

Match the SFM/MVS generated point cloud with 3D panorama scan point cloud.

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

2. Faro Scan
--------------------

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

3. Localization using SIFT
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

We got ideal results. The follwoing image shows the result for the same query image, as the former chapter.

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

**Failed Cases** :

.. image:: images/sg_failed.png
  :align: center

We could still fail, if too much plants points show up. To overcome this we need to retrain the feature extraction
and matching algorithms based on our specified data.

**Summary** :

* We get much more robust feature extraction and matching.
* **TODO** need to refine the pose refinement process.
* **TODO** the performance of the pretrained NetVLAD is not ideal, better to train in our dataset.
