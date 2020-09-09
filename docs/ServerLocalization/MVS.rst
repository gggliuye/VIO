Dense Reconstruction
===============================


1. Using Lidar Scan
----------------------

As shown in the Chapter of "Super Panorama", we get a model for total three floors of the "winter garden" scene
with about 1,000,000 faces. with reasonable quality.


2. Colmap MVS
------------------------

* Using the colmap MVS results (using Patch Match algorithm).
* Modeling with poisson reconstrcution.
* Simplify the model using Meshlab Quadric Edge Collapse Decimation.

We have a quiet satisfying result in our garden scene (one layer garden part, built with about 1200 images).
It has about 11,000,000 faces before simplification.

.. image:: resonstructions/colmap_poisson.png
  :align: center
