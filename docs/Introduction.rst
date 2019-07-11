

Introduction of SLAM
===================================

`Link <https://vio.readthedocs.io/en/latest/index.html>`_

** Simultaneous Localization and Mapping **


SLAM is a complex system.

In most SLAM structure design, the whole system will be seperated into three threads : Tracking, Local Mapping, and Global Mapping.

In a complete SLAM framework, the three threads should run together, but with different frequences.
For an example, for ORBSLAM, Tracking thread will run for each input frame, Local Mapping will run when new keyframe is selected, and Global Mapping will be run when a loop is detected.

.. image:: images/3.jpg
   :height: 400px
   :width: 200 px
   :scale: 50 %
   :alt: JOJO image import test
   :align: center

What is it?
------------------

Two basic questions:
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 * where am I?
 * What is the environment around me?

Sensor
>>>>>>>>>>>>>>>>>>>>>>>>>>>
Robots(or devices) use sensor to "feel" the outside world.

* inner sensor : IMU(acc, gyr), encoder, etc.
* outter sensor : camera, lidar, trace, etc.

Some sensors are limitted by the environment. (GPS by the buildings, Marker and traces need flat environment to be set).
So we need some stable sensor, to offer stable information

Camera
>>>>>>>>>>>>>>>>>>>>>>>
Receive images as a frequence, and generate videos (normally 30 FPS).

* Monocular camera.
* Stereo camera (Two-view as human eyes to offer distances) (two view geometry).
* Depth camera (RGBD). It uses more sensor to offer a depth image in addition (physics methods).
* Other: Event camera, panorama camera, etc.

A simple camera has no scale info, which is essential for computer vision.






Usage
---------


Hand hold device localization 
>>>>>>>>>>>>>>>>>>>>>>>>>
For example, localize a camera, to help guide the user


Self Drive localization 
>>>>>>>>>>>>>>>>>>>>>>>>>>
They will use more sensor, as high accurate IMU, GPS, LIDAR, and camera.


AR Argumented Reality
>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Use camera to localize and make a map, to place virtual objects and interact with them.

