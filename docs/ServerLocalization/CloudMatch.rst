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

However the Faro device is a Lidar dominated device, it lacks a fine calibration of the panorama image.
(For an example, it doesn't have a uniform panorma image size, and it dosen't follow the upper camera model)

We also tried to use the raw image from faro, unfortunately it is of bad quality.
* It is captured by a spinning device -> lack of uniform exterior calibration.
* About 40% of the image is consist of the outer shell of the Lidar Device.
* It is registered by the lidar scan data -> hard for us to calibration.

As a result, we decide to add an additional camera to the Faro device to get a stable image rescource.
