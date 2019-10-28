Lidar SLAM
=========================


LOAM
-----------------------

`LOAM <https://github.com/laboshinl/loam_velodyne>`_


A-LOAM
-------------------------


LeGO LOAM
--------------------


HDL-GRAPH-SLAM
----------------------

IMLS-SLAM
-------------------
Implicit Moving Least Squares(IMLS) surface represetation is used to handle large amount and sparisty of acquired data. It makes me proud to see some schoolmates made such a contributional work.

Pretreatment
~~~~~~~~~~~~~~~~~~
1. Unwarp lidar scan, It is done b linear interploation using the last relative pose. 

.. math::
    \bar{\tau}(t_{k}) = \tau(t_{k-1}) * \tau(t_{k-2})^{-1} * \tau(t_{k-1})
    
    \tau(t) = Interpolation(\bar{\tau}(t_{k}), \tau(t_{k-1}), t), t \subseteq (t_{k-1}, t_{k})


2. remove small size segmented point cloud, and grouped cloud with small bounding box.
