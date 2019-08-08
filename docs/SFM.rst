SFM
====================

SfM (structure from motion), is an other useful tool for localization and mapping. 
Basicly, SfM uses the same technique as SLAM system : feature point extraction, matching algorithms, multiple view geometry, and non linear optimization(bundle adjustment). 
At the same time SfM and SLAM have many differences, mostly in their pipeline (SLAM uses a realtime pipeline, however SfM has three main types: **incremental** , **global**  and **hierarchical** ) and some algorithm details.

* Incremental SfM has the most close structure to SLAM. It is fast and robust. 
* Global SfM is a process, try to initialize all camera posese, and triangulation, then do global bundle adjustment directly.
It is more accurate, however takes longer to calculate. And its result dependents heavily on the initialization.
* Hierachical SfM is the between of the upper two methods. It initialize multiply submaps optimization each of them, then take then together.



Here is SFM result built with `colmap <https://colmap.github.io/>`_ , with images taken from Winter Plaze, GuangZhou.

.. image:: images/sfm/1.png
   :scale: 80 %
   :align: center



Triangulation
--------------------

Triangulation is mainly two view geometry, to generate depth with stereocopy view. In **Colmap** , only two view triangulation is used. So we describe triangulation details here.

Suppose we have two observations from two camera view :

.. math::
    \mathbf{x} = P \mathbf{X}, \mathbf{x'} = P' \mathbf{X}

The depths of this point in two views are unknown, however we know that x and X should be in the same line. As a result, their cross production should be a zero vector:

.. math::
    \mathbf{x} \times (P\mathbf{X}) = \mathbf{0}
    
The upper function can be write as :

.. math::
    x(p^{3T}\mathbf{X}) - (p^{1T}\mathbf{X}) = 0 

.. math::
    y(p^{3T}\mathbf{X}) - (p^{2T}\mathbf{X}) = 0 

.. math::
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


Colmap
-------------------------

In my opinion, Colmap is the best open source SFM tools. It offers userfriendly interface, various algorithm options, uniform output structures, and reliable results. Many applications are built based on it. We will introduce some of the basic idea of Colmap, and its pipeline.

Database format
~~~~~~~~~~~~~~~~~~~~~
All the data is saved into a **database.db** file.

* cameras : recording different camera models and all the image associated to it.
* keypoints : float32 values of the (u,v) pixel poisiton + scale factor + orientation (SIFT concentions) + afflinity + affline shape.
* descriptors : uint 8 binary blobs (only offer 128 bits descriptors).
* matches : pair ids + F,E,H blobs

For **Sparse Reconstruction** the data can be saved as "txt" or "bin" files. 

* cameras : cameras' intrinsic parameters
* images : two lines for each image, 1) image Id + quaternion + position + camera Id + name, 2) set of feature points X + Y + point3d Id.
* points :  X + Y + Z + R + G + B + error + set of image id and point2d id.

Pipeline
~~~~~~~~~~~~~~~~~~
* feature extraction -> SIFT
* feature matching
* sparse reconstruction (incremental SfM)
* dense reconstruction (undistort, stereo, Fusion)
* Possion / Delaunay reconstruction.

Feature Matching
~~~~~~~~~~~~~~~~~~~~~~~~

**Exhaustive Matching**:

Every image is matched against every other image, while the block size determines how many images are loaded from disk into memory at the same time.

**Sequential Matching**:

Used for sequential order input images,  e.g., by a video camera. Consecutively captured images are matched against each other.
Will not work for unordered images sets.

**Vocabulary Tree Matching**:

Used to large dataset (several thousands), bag of visual words. It is our normal choose, as it is relatively faster than exhaustive method, and it is robust. Its pipleine is :

* Load the pretrained vocabulary tree.
* IndexImagesInVisualIndex : extracte the top scale features and add the image to vocabulary index. And compute the tf-idf index. We can save the made index for further use. 
* MatchNearestNeighborsInVisualIndex : **runtime** : take about 36 seconds for a data set with 622 images taking maximual 1000 features, in a i5 CPU.

It is still too slow for a half-real-time application, we tried with less features. The system do speeds up, however the results became highly unreliable.

**Spatial Matching**:

This matching mode matches every image against its spatial nearest neighbors(can be set manually, or by default using GPS).

**Transitive Matching**: 

This matching mode uses the transitive relations of already existing feature matches to produce a more complete matching graph. If an image A matches to an image B and B matches to C, then this matcher attempts to match A to C directly.

**Custom Matching**


MVS(Multiple View Stereovision)
--------------------------------

Its objective is to build a denser map, using multiply view geometry. Made for dense reconstrcution from images only, which will not be helpful for our localization propose.

Here is the MVS fusion result built with `colmap <https://colmap.github.io/>`_ , with images taken from Winter Plaze, GuangZhou.

   
.. image:: images/sfm/7.png
   :scale: 80 %
   :align: center

