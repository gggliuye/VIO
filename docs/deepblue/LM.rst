VINS code analysis
=========================

Analysis the structure and the details of `VINS <https://github.com/HKUST-Aerial-Robotics/VINS-Mono>`_ code. Take the android implement as example.

Main Node
----------------------------
In **naive-lib.cpp** all the intereface plugin functions are defined.

The main node in VINS android is : **Estimator_node**. Where many fronter functions are defined : functions to run data set test, functions to run in real time. Mainly, we have two functions **receive_IMU** and **receive_img**, they will call **img_callback**, **imu_callback**, and **feature_callback** respectively.

receive_img 

     -> img_callback : call this fcn when receive image
     
          -> proprecessing : gestion of frequence / error rejcetion judgements
          
          -> prcess the image (seperate the process of stereo camera and mono camera)
          
          -> calculate feature points ( **FeatureTracker** )-> call feature_callback
          
          -> draw track messages to image (tracked points in green, not tracked points in red, etc)
          
     -> draw_mainui : call **DrawResult** class, to draw AR rendering(drawAR) or draw trajectory with map points(Reprojection)
     
     -> add more debug infomation to shown image

receive_IMU

     -> imu_callback :
     
          -> add to query 
          
          -> predict bu intergration



Grid table:

+------------+------------+-----------+ 
| Header 1   | Header 2   | Header 3  | 
+============+============+===========+ 
| body row 1 | column 2   | column 3  | 
+------------+------------+-----------+ 
| body row 2 | Cells may span columns.| 
+------------+------------+-----------+ 
| body row 3 | Cells may  | - Cells   | 
+------------+ span rows. | - contain | 
| body row 4 |            | - blocks. | 
+------------+------------+-----------+





Levenberg-Marquardt Method
-----------------------


.. math::

    y = c \cdot e^{a \cdot x} + d \cdot e^{b \cdot x}
 
.. math::

    Residual = \sum_{i} (c \cdot e^{a \cdot x_{i}} + d \cdot e^{b \cdot x_{i}} - \overline{y_{i}} )


.. math::

    J = [ x_{i}*c*e^{a*x_{i}}  , x_{i}*d*e^{b*x_{i}}, e^{a*x_{i} , e^{b*x_{i}} ]
   
