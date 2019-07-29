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



Triangulation
--------------------

Suppose we have two observations from two camera view :

.. math::
    \mathbf{x} = P \mathbf{X}, \mathbf{x'} = P' \mathbf{X}

The depths of this point in two views are unknown, however we know that x and X should be in the same line. As a result, their cross production should be a zero vector:

.. math::
    \mathbf{x} \times (P\mathbf{X}) = \mathbf{0}
    
The upper function can be write as :

.. math::
    x(p^{3T}\mathbf{X}) - (p^{1T}\mathbf{X}) = 0
    y(p^{3T}\mathbf{X}) - (p^{2T}\mathbf{X}) = 0
    x(p^{2T}\mathbf{X}) - y(p^{1T}\mathbf{X}) = 0

We can find that the thrid function is redundant (as it can be derivated from the first two functions).
If we have two points, we can rewrite the problem to be a linear optimization problem:

.. math::
    \mathbf{AX} = 0
    
.. math::
    \mathbf{A} = \begin{bmatrix}
    xp^{3T} - p^{1T} \\  yp^{3T} - p^{2T} \\ x'p'^{3T} - p'^{1T} \\  y'p'^{3T} - p'^{2T} 
    \end{bmatrix}

**Homogeneous method (DLT)**

The problem becomes:

.. math::
    \min \lVert \mathbf{AX} \Vert ^{2} =  \min \mathbf{X^{T}A^{T}AX}
   
From basic in Calculus, we know that the solution must fulfill:

.. math::
    \triangledown \mathbf{X^{T}A^{T}AX} = \gamma \triangledown \mathbf{X^{T}X}  \Leftrightarrow
    2\mathbf{A^{T}AX} = \gamma 2 \mathbf{X}
    \Leftrightarrow \mathbf{A^{T}AX} = \gamma \mathbf{X}

Because of the special shape of ATA we compute the eigenvectors efficiently using the so called Singular Value
Decomposition (SVD).

.. math::
    A = USV^{T}

where U and V are orthogonal, and S is diagonal matrix. Then,

.. math::
    A^{T}A = (USV^{T})^{T}USV^{T}= VS^{T}U^{T}USV^{T} =  VS^{T}SV^{T}

The diagonal elements of STS are ordered decreasingly σ1^2, σ2^2, σ3^2, ...
Thus, to find an eigenvector corresponding to the smallest eigenvalue we should select the last column of V.

