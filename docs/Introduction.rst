

Introduction of SLAM
===================================

`Link <https://vio.readthedocs.io/en/latest/index.html>`_

** Simultaneous Localization and Mapping **


SLAM is a complex system.

In most SLAM structure design, the whole system will be seperated into three threads : Tracking, Local Mapping, and Global Mapping.

In a complete SLAM framework, the three threads should run together, but with different frequences.
For an example, for ORBSLAM, Tracking thread will run for each input frame, Local Mapping will run when new keyframe is selected, and Global Mapping will be run when a loop is detected.


.. image:: images/3.jpg
   :height: 100px
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

