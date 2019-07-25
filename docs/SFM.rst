SFM
====================

SfM (structure from motion), is an other useful tool for localization and mapping. 
Basicly, SfM uses the same technique as SLAM system : feature point extraction, matching algorithms, multiple view geometry, and non linear optimization(bundle adjustment). 
At the same time SfM and SLAM have many differences, mostly in their pipeline (SLAM uses a realtime pipeline, however SfM has three main types: incremental, global and hierarchical) and some algorithm details (which can be seen  ` SLAMvsSFM <https://vio.readthedocs.io/en/latest/UnitySc/unityShadowCreator.html#sfmslam>`_ )


Here is SFM result built with `colmap <https://colmap.github.io/>`_ , with images taken from Winter Plaze, GuangZhou.

.. image:: images/sfm/1.png
   :scale: 80 %
   :align: center



MVS(Multiple View Stereovision)
--------------------------------

Here is the MVS fusion result built with `colmap <https://colmap.github.io/>`_ , with images taken from Winter Plaze, GuangZhou.

   
.. image:: images/sfm/7.png
   :scale: 80 %
   :align: center
