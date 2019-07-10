

Introduction of SLAM
===================================


SLAM is a complex system.

In most SLAM structure design, the whole system will be seperated into three threads : Tracking, Local Mapping, and Global Mapping.

In a complete SLAM framework, the three threads should run together, but with different frequences.
For an example, for ORBSLAM, Tracking thread will run for each input frame, Local Mapping will run when new keyframe is selected, and Global Mapping will be run when a loop is detected.


