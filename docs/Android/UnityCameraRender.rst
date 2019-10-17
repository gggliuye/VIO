Unity Camera Renderer
================================

When making a Unity AR application, the basic problem is the structure of the process. And here we will talk about it.

The whole process of a AR application should be about:

* open camera
* receive camera frame (and other sensor data)
* process the received camera frame (and other sensor data)
* output the result
* render the view

The first problem is to assign the missions to suitable people. For our application, we have assigned the rendering task to Unity3D. However, we have not decide the others yet.
I will start from the very first step : **open camera**.

Open Camera
~~~~~~~~~~~~~~~~~~~~~

For this task, we have three choices. 

* Unity open the camera
* Android Java open the camera
* C++ call opencv to open the camera









