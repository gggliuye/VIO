VIO deep blue
-------------------------

**Question 1**: At a certain time, the cameras and landmarks seen of a SLAM system can be seen as the image below. xi is the camera pose, L represents the observed landmark. When a landmark k was seen by ith camera in the world frame, the reprojection error is noted as r(xi_i, L_k)

.. image:: images/week4_1.PNG
   :align: center

**1.1 the information matrix of the upper system** :

As a result, we have seven elements in the reprojection error term:

.. math:: 
    r_{1,1} = r(\xi_{1}, L_{1}) & r_{1,2} = r(\xi_{1}, L_{2})
    r_{2,1} = r(\xi_{2}, L_{1}) & r_{2,2} = r(\xi_{2}, L_{2}) & r_{2,3} = r(\xi_{2}, L_{3})
    r_{3,1} = r(\xi_{3}, L_{1}) & r_{3,2} = r(\xi_{3}, L_{2})
