VINS code
=========================

Analysis the structure and the details of `VINS <https://github.com/HKUST-Aerial-Robotics/VINS-Mono>`_ code. Take the android implement as example. As the original code is based on ROS(robot opeartion system), changes are made to create virtual ROS message, and manual call the ros callback functions. It has two main threads: **vins_estimator** and **loop_fusion**.

Top Node
----------------------------
In **naive-lib.cpp** all the intereface plugin functions are defined. In the android version code, there are lots of management of memory(mutex), query and buffer, and lots of global variables. It is the result of the absent of ROS, we need to take great consideration of these parts. 

The main node in VINS android is : **Estimator_node**. Where many fronter functions are defined : functions to run data set test, functions to run in real time. In the init() function , it starts three threads : **process**, **loop_detection**, **pose_graph**.
The main thread in ROS node is **process** it calls two essential functions **receive_IMU** and **receive_img**, they will call **img_callback**, **imu_callback**, and **feature_callback** respectively. And it is also responsible for loop deteciton and loop closure.

call from other
~~~~~~~~~~~~~~~~~~~~~~~

**receive_img**  (called in java plugins, used to show image in android application)

     -> img_callback : call this fcn when receive image
     
          -> proprecessing : gestion of frequence / error rejcetion judgements
          
          -> process the image (seperate the process of stereo camera and mono camera)
          
          -> calculate feature points ( **FeatureTracker** )-> call feature_callback (push the feature to buffer) 
          
          -> draw track messages to image (tracked points in green, not tracked points in red, etc)
          
     -> draw_mainui : call **DrawResult** class, to draw AR rendering(drawAR) or draw trajectory with map points(Reprojection)
     
     -> add more debug infomation to shown image

**receive_IMU** (called in **Phone Sensor**)

     -> imu_callback :
     
          -> add to process query 
          
          -> predict the current state (position, quaternion and velocity) by intergration (mean value). 

Details of mean value integration (in estimator_node -> predict()), where all the values are vectors:

.. math::
    \begin{cases}
    p_{k+1}  = p_{k} + v_{k} \delta t + \frac{1}{2} \bar{a}^{w} (\delta t)^{2}   \\
    v_{k+1} = v_{k} + \bar{a}^{w} \delta t   \\
    q_{k+1} = q_{k} \otimes \begin{bmatrix} 1 \\  \frac{1}{2}  \bar{\omega}  \delta t \end{bmatrix}  
    \end{cases}

.. math::
    \bar{\omega} = \frac{1}{2} (\omega_{k+1} + \omega_{k}) - b_{gyro} 

.. math::
    \bar{a}^{w} = \frac{1}{2} ( q_{k}(a_{k}^{b} - b_{acc}) + q_{k+1}(a_{k+1}^{b} - b_{acc}) ) - g_{k}


**Queation** : these results are never used, it is real necessary??

Process
~~~~~~~~~~~~~~~~~~~~~~~~~

* call sendIMU -> estimator.processIMU
* call estimator. processImage

**loop closure**
there are also loop closure process in this thread.

**Queation** we also have loop closure in process_loop_detection thread. Is is redundant??


process_loop_detection
~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Take the first image from the keyframe buffer to process.
* Add the keyframe to **KeyFrameDatabase**.
* Extract Brief descriptors of the keyframe features.
* Start Loop Closure (**LoopClosure** class). if success, receive the looped kerframe's index. 
* if far enough, add this loop to process query<int> (optimize_posegraph_buf) : calculate the matches between current processing keyframe and the looped keyframe, then Pnp to get pose.
* if too many keyframe in database, downsample erase some keyframes.


process_pose_graph
~~~~~~~~~~~~~~~~~~~~~~~~~

* if we have loop in the query (optimize_posegraph_buf, as we may add loop in process_loop_detection thread).
* do optimize4DoFLoopPoseGraph (cerse solver, 4DOF as VINS has set the gravity direction to be vertical)
* update infomation for visualization
  
  
VINS estimator
----------------------------

Method called above in "top node" : estimator.processIMU, estimator.processImage, estimator.retrive_data_vector .  Its basic idea is to manage a **slide window** , make imu preintegration and imu observation, also marginalization, etc.

processIMU
~~~~~~~~~~~~~~~~~~~

A **IntegrationBase** class is made for pre-intergration management and calculation.

**IntegrationBase**
:::::::::::::::::::::::

* push back a new measurment : timestamp, gyrocope measure, and accelerometer measure. Add them to the buffer and **propagate** the system.
* midPointIntegration : basic it is the same expression as above, about we are doing integration for the **error term of preintegration** here (as a result, n gravity term here). (in the VINS source code, they note p, v, and q, however I found it being misleading, so I note them as alpha , beta and gamma as in [#]_ ).

.. math::
    \begin{cases}
    \alpha_{k+1}  = \alpha_{k} + \beta_{k} \delta t + \frac{1}{2} \bar{a}^{w} (\delta t)^{2}   \\
    \beta_{k+1} = \beta_{k} + \bar{a}^{w} \delta t   \\
    \gamma_{k+1} = \gamma_{k} \otimes \begin{bmatrix} 1 \\  \frac{1}{2}  \bar{\omega}  \delta t \end{bmatrix}  
    \end{cases}

.. math::
    \bar{\omega} = \frac{1}{2} (\omega_{k+1} + \omega_{k}) - b_{gyro} 

.. math::
    \bar{a}^{w} = \frac{1}{2} ( \gamma_{k}(a_{k}^{b} - b_{acc}) + \gamma_{k+1}(a_{k+1}^{b} - b_{acc}) )
    
* **Jacobian update** : (it is optinal, normally set true) three matrix are calculated before to fasten. Noise is seen as gaussian. And the F matrix(15*15) and the error term propagation matrix V (15*18) are calculated. (remember to normalize quaternion). In the end, two 15*15 matrix : Jacobian and Covariance are calculated.

.. math::
    [R_{\omega}]_{X} = [ \bar{\omega} ]_{X} , 
    [R_{\tilde{a}_{k}}]_{X} = [a_{k}^{b} - b_{acc}]_{X},
    [R_{\tilde{a}_{k+1}}]_{X} = [a_{k+1}^{b} - b_{acc}]_{X}
 
.. math::
    R_{k} \leftarrow q_{k} , R_{k+1} \leftarrow q_{k+1}
    
Jacobian is (noted as F):

.. math::
    \begin{bmatrix}
    I_{3 \times 3} & f_{12} & I_{3 \times 3} \delta t  & f_{14} & f_{15} \\
    0_{3 \times 3} & I -[R_{\omega}]_{X} \delta t & 0_{3 \times 3} & 0_{3 \times 3} & -I_{3 \times 3} \delta t \\
    0_{3 \times 3} & f_{32} & I_{3 \times 3} & f_{34} & f_{35} \\
    0_{3 \times 3} & 0_{3 \times 3} & 0_{3 \times 3} & I_{3 \times 3} & 0_{3 \times 3} & \\
    0_{3 \times 3} & 0_{3 \times 3} & 0_{3 \times 3} & 0_{3 \times 3} & I_{3 \times 3}
    \end{bmatrix}
    
.. math::
    f_{12} = - \frac{1}{4} R_{k} [R_{\tilde{a}_{k}}]_{X} (\delta t)^{2} - \frac{1}{4} R_{k+1} [R_{\tilde{a}_{k+1}}]_{X} (I - [R_{\omega}]_{X} \delta t) (\delta t)^{2}

.. math::
    f_{14} = - \frac{1}{4} ( R_{k} + R_{k+1} ) (\delta t)^{2}
    
.. math::  
    f_{15} = - \frac{1}{4} R_{k+1} [R_{\tilde{a}_{k+1}}]_{X}  (\delta t)^{2} (- \delta t)
    
.. math::
    f_{32} = - \frac{1}{2} R_{k} [R_{\tilde{a}_{k}}]_{X} \delta t - \frac{1}{2} R_{k+1} [R_{\tilde{a}_{k+1}}]_{X} (I - [R_{\omega}]_{X} \delta t) \delta t

.. math::
    f_{34} = - \frac{1}{2}( R_{k} + R_{k+1} )\delta t

.. math::
    f_{35} = - \frac{1}{2} R_{k+1} [R_{\tilde{a}_{k+1}}]_{X} \delta t (-\delta t)
    
The noise term matrix is (noted as V):
    
.. math::
    \begin{bmatrix}
    \frac{1}{4} R_{k}  (\delta t)^{2} & g_{12} & \frac{1}{4} R_{k+1}  (\delta t)^{2} \delta t & g_{14} & 0_{3 \times 3} & 0_{3 \times 3} \\
    0_{3 \times 3} & \frac{1}{2} I_{3 \times 3} \delta t & 0_{3 \times 3} & \frac{1}{2} I_{3 \times 3} \delta t & 0_{3 \times 3} & 0_{3 \times 3} \\
    \frac{1}{2} R_{k} \delta t & g_{32} & \frac{1}{2} R_{k+1} \delta t & g_{34} & 0_{3 \times 3} & 0_{3 \times 3} \\
    0_{3 \times 3} & 0_{3 \times 3} & 0_{3 \times 3} & 0_{3 \times 3} & I_{3 \times 3} \delta t & 0_{3 \times 3} & \\
    0_{3 \times 3} & 0_{3 \times 3} & 0_{3 \times 3} & 0_{3 \times 3} & 0_{3 \times 3} & I_{3 \times 3} \delta t
    \end{bmatrix}

.. math::
    g_{12} = g_{14} = - \frac{1}{4} R_{k+1} [R_{\tilde{a}_{k+1}}]_{X} (\delta t)^{2} (\frac{1}{2} \delta t)

.. math::
    g_{32} = g_{34} = - \frac{1}{2} R_{k+1} [R_{\tilde{a}_{k+1}}]_{X} \delta t (\frac{1}{2} \delta t)
    
This is a iteration process, as we can see below, as a result, the Jacobian is the acculumation of F.

.. math::
    \delta x_{k+1} = F_{k+1} \delta x_{k} = F_{k+1} F_{k} ... F_{1} \delta x_{0} = J_{k+1} \delta x_{0}

.. math::
    Jacobian_{k+1} = F_{k+1} * Jacobian_{k}

.. math::
    Covariance_{k+1} = F_{k+1} * Covariance_{k} * F_{k+1}^{T} + V_{k+1} * Noise * V_{k+1}^{T}
    
* **evaluate** : calcuates the residual (15*1 vector)


* also have checkJacobian : to check the calculation of jacobian of the system;  offer an option of eulerIntegration (however it is less precise than mid point integration); and compare the results of mid point integration and euler integration.


**Integration** 
:::::::::::::::::::

In the final part of processIMU, the integration terms of the real world **physics variables** are calculated as below, where j indicates ith window, k indicates kth imu data (between two received image). 
    
.. math::
    \begin{cases}
    P_{j,k+1} = P_{j,k} + V_{j,k} \delta t + \frac{1}{2} \bar{a}_{j,k+1}^{w} (\delta t)^{2}  \\
    V_{j,k+1} = V_{j,k} + \bar{a}_{j,k+1}^{w} \delta t  \\
    Q_{j,k+1} = Q_{j,k} \otimes \begin{bmatrix} 1 \\  \frac{1}{2}  \bar{\omega}  \delta t \end{bmatrix}
    \end{cases}
    
.. math::
    \bar{a}_{j,k+1}^{w} = \frac{1}{2}(Q_{j,k} (a_{j,k}^{b} - b_{acc,j})  + Q_{j,k+1} (a_{j,k+1}^{b} - b_{acc,j}) ) - g^{w}

.. math::
    \bar{\omega}_{j,k+1} = \frac{1}{2} (\omega_{k+1} + \omega_{k}) - b_{gyro,j} 


processImage
~~~~~~~~~~~~~~~~~~~~~~~

**Pipeline**:

* **addFeatureCheckParallax** check the image simliarity, to choose whether **marginalize** the oldest image in the window(to make space for the new coming , and the current image is treated as new keyframe) or the last image in the window (if the recent images are similar).
* create new image frame, and create the image pre-integration base.
* option : ( ESTIMATE_EXTRINSIC == 2 ) calibrate the extrinsic parameters.
* (solver_flag == INITIAL) -> fill the slide window and try to initialize **initialStructure**.
* (solver_flag == NON_LINEAR) -> initialize success, manage the slide window.


initialStructure
~~~~~~~~~~~~~~~~~~~~~~~~~~

* check IMU state. where Delta V is the result of preintegration between two frames in integration base, Delta t is the time interval between frames.

.. math::
    \bar{g} = \frac{1}{Size_{window}} \sum_{window} \frac{\Delta v} {\Delta t}

.. math::
    \Delta g = \frac{\Delta v}{\Delta t} - \bar{g}
    
.. math::
    Var = \sqrt{ \frac{1}{Size_{window}} \sum_{window} (\Delta g)^{T} (\Delta g)  }

if Var < 0.25 : "IMU excitation not enouth!"

* initialize a sfm features vector by **FeatureManager** .
* check the relative pose, if not enough features or parallax, ask to move the device.
* **GlobalSFM** construct.
* if global sfm succeed, solve PnP for all frames.
* solve odometry and manage slide window
* visualInitialAlign.   VisualIMUAlignment

solveOdometry
~~~~~~~~~~~~~~~~~~~~

* f_manager.triangulate
* optimization()

slideWindow
~~~~~~~~~~~~~~~~~~~~~

* slideWindowOld : (solver_flag == NON_LINEAR ? true : false) f_manager.removeBackShiftDepth,  f_manager.removeBack
* slideWindowNew : f_manager.removeFront

optimization
~~~~~~~~~~~~~~~~~~~~~~~~~~


use ceres to optimize : CauchyLoss

* add pose local parameter block (of the slide window)
* add current frame pose block
* add residual of imu preintegrations (of the slide window) 
* add feature residual  (ESTIMATE_TD option)
* marginalization_info->addResidualBlockInfo of the upper resiudal

linear_solver_type set to ceres::DENSE_SCHUR, trust_region_strategy_type set to ceres::DOGLEG.

Slide window marginalization.

* marginalization_info->preMarginalize();
* marginalization_info->marginalize();



Marginalization
---------------------

ResidualBlockInfo
~~~~~~~~~~~~~~~~~
Evaluate : evaluate the cerse loss evaluate.

.. math::
    r = \lVert \mathbf{r} \rVert_{2} 

.. math::
    \alpha = 1 - \sqrt{  1 + 2  r \frac{rho_{2}}{rho_{1}} }

.. math::
    r_{scaled} =  \frac{ \sqrt{rho_{1}} }{1- \alpha}

.. math::
    \mathbf{J} \leftarrow \sqrt{rho_{1}} (\mathbf{J} - \frac{\alpha}{r} \mathbf{r} (\mathbf{r}^{T} \mathbf{J}))


ThreadsStruct, 

MarginalizationInfo, 

MarginalizationFactor



Feature Manager
---------------------
* list<FeaturePerId> feature
* vector<FeaturePerFrame> feature_per_frame

.. [#] Sola J. Quaternion kinematics for the error-state Kalman filter[J]. arXiv preprint arXiv:1711.02508, 2017.

