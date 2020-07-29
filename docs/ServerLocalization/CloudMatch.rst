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
