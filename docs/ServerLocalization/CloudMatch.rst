Cloud Match
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

Using the `code <https://github.com/gggliuye/VIO/blob/master/pretreatment/panorama_extraction_perfect_sphere.ipynb>`_
We can get the aligned depth image :

.. image:: images/pano_faro.PNG
  :align: center

And some sample of pinhole camera images:

.. image:: images/pinhole_faro.PNG
  :align: center
