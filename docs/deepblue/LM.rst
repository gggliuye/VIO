VINS code analysis
=========================

Analysis the structure and the details of `VINS <https://github.com/HKUST-Aerial-Robotics/VINS-Mono>`_ code. Take the android implement as example. As the original code is based on ROS(robot opeartion system), changes are made to create virtual ROS message, and manual call the ros callback functions. It has two main threads: **vins_estimator** and **loop_fusion**.

Top Node
----------------------------
In **naive-lib.cpp** all the intereface plugin functions are defined.

The main node in VINS android is : **Estimator_node**. Where many fronter functions are defined : functions to run data set test, functions to run in real time. In the init() function , it starts three threads : **process**, **loop_detection**, **pose_graph**.
The main thread in ROS node is **process** it calls two essential functions **receive_IMU** and **receive_img**, they will call **img_callback**, **imu_callback**, and **feature_callback** respectively. And it is also responsible for loop deteciton and loop closure.

Process
~~~~~~~~~~~~~~~~~~~~~~~~~

**receive_img**  (called in java plugins)

     -> img_callback : call this fcn when receive image
     
          -> proprecessing : gestion of frequence / error rejcetion judgements
          
          -> prcess the image (seperate the process of stereo camera and mono camera)
          
          -> calculate feature points ( **FeatureTracker** )-> call feature_callback
          
          -> draw track messages to image (tracked points in green, not tracked points in red, etc)
          
     -> draw_mainui : call **DrawResult** class, to draw AR rendering(drawAR) or draw trajectory with map points(Reprojection)
     
     -> add more debug infomation to shown image

**receive_IMU** (called in **Phone Sensor**)

     -> imu_callback :
     
          -> add to process query 
          
          -> predict the current state (position, quaternion and velocity) by intergration (mean value)

Details of mean value integration (in estimator_node -> predict()), where all the values are vectors:

.. math::
    p_{k+1}  = p_{k} + v_{k} \delta t + \frac{1}{2} \bar{a}_{k} (\delta t)^{2}
    
.. math::
    v_{k+1} = v_{k} + \bar{a}_{k} \delta t 

.. math::
    q_{k+1} = q_{k} \otimes \begin{bmatrix} 1 \\  \frac{1}{2}  \bar{\omega}  \delta t \end{bmatrix}

.. math::
    \bar{\omega} = \frac{1}{2} (\omega_{k+1} + \omega_{k}) - b_{gyro} 

.. math::
    \bar{a} = \frac{1}{2} ( q_{k}(a_{imu,k} - b_{acc}) + q_{k+1}(a_{imu,k+1} - b_{acc}) ) - g_{k}


process_loop_detection
~~~~~~~~~~~~~~~~~~~~~~~~~~~



process_pose_graph
~~~~~~~~~~~~~~~~~~~~~~~~~


   
