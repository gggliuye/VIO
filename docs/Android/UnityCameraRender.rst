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

They have different advantages and drawbacks:

* If Unity open the camera, we need to pass the image data to C++ to do further calcualtion. We need to define more plugins to realize it. As a result, our program will become unsafe, and the processing will become less efficient (as it will involve more memory gestion). However it is the fastest way to develop. As it is extremely easy to open camera with unity's "CameraTexture" class.
* All we can use Android java to opencv camera, and set the frame callback to be our C++ function. As a result, we need to build another Java plugin package along with the C++ plugin. And we need to implement the renderer plugin to avoid too much memory use. But we can still easily open camera and manger it with "android.hardware.camera".
* The third one is to C++ code in the whole process. Open camera with Opencv C++, process the image, and render image with unity low level plugin. We should have one single C++ plugin library. However, we need include dependence of Opencv (or we need to implement the whole image receive and decode process, which is very hard).

We used the first method in the very beginning of our project. And as our projcet precceding, I begin to try to realize the second method. It takes me a few days to finish it (with almost no Java experience, and limited unity exprience, but robust C++ fundation). 


Whole Process
~~~~~~~~~~~~~~~~~~~~~~~~~~
The whole process structure and a time circle is shown here.

.. image:: structure.PNG
   :width: 80%
   :align: center
   
.. image:: lifecircle.PNG
   :width: 80%
   :align: center

Android Java Camera Surface
~~~~~~~~~~~~~~~~~~~~~~~

The camera is opened with android camera surface. With three main function :

* **public void initialize(int presetWidth, int presetHeight, long handlerPtr);** Initialize the camera , set the layout, and link the C++ "C++ handler"'s pointer. Inputs are the camera stream width and height, and the pointer to the handler.
* **public void receiveCameraFrame(byte[] data, int width, int height, boolean backCamera);** This will call the callback function when receving a new image frame. The inputs are the image data, the image width and height, a bool to indicate the front/back camera.
* **public  native void setCameraFrame(byte[] paramArrayOfByte, int width, int height, long handlerPtr);** This is the callback function (which is written in C++, but use plugin to introduce here), it will go to the "C++ handler", and call the function defined in the "C++ handler" to process the image (algorithm calculation and low level rendering). The inputs are the image data array, the width and height of the image, and the pointer to the handler.

C++ Plugin
~~~~~~~~~~~~~~~~~~~~~

