Introduction of SLAM
===================================

`Link of the Site <https://vio.readthedocs.io/zh_CN/latest/index.html>`_


SLAM
-----------------------------

SLAM **Simultaneous Localization and Mapping** is a complex system.

In most SLAM structure design, the whole system will be seperated into three threads : Tracking, Local Mapping, and Global Mapping.

.. image:: images/pipeline.png
   :align: center

In a complete SLAM framework, the three threads should run together, but with different frequences.
For an example, for ORBSLAM, Tracking thread will run for each input frame, Local Mapping will run when new keyframe is selected, and Global Mapping will be run when a loop is detected.

.. image:: images/Slam.png
   :scale: 100 %
   :alt: image import test
   :align: center

Probability problem
--------------------------
SLAM is actully a probability problem, given a series of control "u", sensor observation "o", over a discrete time step "t" to compute an estimate of the state "x". (for SLAM, given image and other sensors, to estimate camera pose "p" and the map "m").
Also, the system has noise, which will be modeled as Gaussian distribution in most cases. If using the probabilistic model, we can write the observation model to be :

.. math::
    P(\mathbf{o}_{t+1} \mid \mathbf{x}_{t+1}, \mathbf{u}_{1:t})

And our objective is to estimate the state given obeservation and contrl:

.. math::
    P( \mathbf{x}_{t+1} \mid \mathbf{u}_{1:t} , \mathbf{o}_{1:t+1} )

MAP
~~~~~~~~~~~~~~~~

Normally, we do not have any control in a SLAM system, we can ignore "u" here. Knowing that all the observation are independent to each other. Then, applying Bayes' rule:

.. math::
    P( \mathbf{x}_{t+1} \mid \mathbf{o}_{1:t+1} ) =
    \sum_{i=1}^{t+1} P( \mathbf{x}_{t+1} \mid \mathbf{o}_{i} ) =
    \sum_{i=1}^{t+1}\frac{P(\mathbf{o}_{i} \mid  \mathbf{x}_{t+1} ) P(\mathbf{x}_{t+1}) }{P(\mathbf{o}_{i})}

Ignoring the down mark "t":

.. math::
    P( \mathbf{x} \mid \mathbf{o} ) = \frac{P(\mathbf{o} \mid  \mathbf{x} ) P(\mathbf{x}) }{P(\mathbf{o})}

Using **MAP** (Maximum a posteriori estimation) the best estimation of the system become:

.. math::
    \bar{\mathbf{x}_{MAP}} = arg \max _{x}  P( \mathbf{x} \mid \mathbf{o} )

EM
~~~~~~~~~~~~~~~~~~~~
As the system state can be divide into two parts : map "m" and camera pose "p".
Another modelization of the system can be seen in `wiki <https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping#Problem_definition>`_ by alternating updates of the two beliefs in a form of EM algorithm (Expectation–maximization algorithm).

.. math::
    P( \mathbf{x}_{t+1} \mid \mathbf{u}_{1:t} , \mathbf{o}_{1:t+1} ) = P( \mathbf{p}_{t+1}, \mathbf{m}_{t+1} \mid \mathbf{u}_{1:t} , \mathbf{o}_{1:t+1} )

.. math::
    P(\mathbf{p}_{t} \mid \mathbf{m}_{t} , \mathbf{u}_{1:t} , \mathbf{o}_{1:t} ) = \sum_{m_{t-1}} P(\mathbf{o}_{t} \mid \mathbf{p}_{t},\mathbf{m}_{t}, \mathbf{u}_{1:t}) \sum_{p_{t-1}} P(\mathbf{p}_{t} \mid \mathbf{p}_{t-1}) P(\mathbf{p}_{t-1} \mid \mathbf{m}_{t}, \mathbf{o}_{1:t-1}, \mathbf{u}_{1:t}) / Z

.. math::
    P(\mathbf{m}_{t} \mid \mathbf{p}_{t} \mathbf{u}_{1:t} , \mathbf{o}_{1:t} ) = \sum_{p_{t}} \sum_{m_{t}} P(\mathbf{m}_{t} \mid \mathbf{p}_{t}, \mathbf{m}_{t-1}, \mathbf{o}_{t}, \mathbf{u}_{1:t}) P(\mathbf{m}_{t-1} ,\mathbf{p}_{t} \mid \mathbf{o}_{1:t-1}, \mathbf{m}_{t-1}, \mathbf{u}_{1:t})

Optimization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. math::
    \bar{\mathbf{x}_{MAP}} = arg \max _{x}  \frac{P(\mathbf{o} \mid  \mathbf{x} ) P(\mathbf{x}) }{P(\mathbf{o})}
                     = arg \max _{x}  P(\mathbf{o} \mid  \mathbf{x} ) P(\mathbf{x})

Take a minus Log of the function (for better process Gaussian distribution):

.. math::
    \bar{\mathbf{x}_{MAP}} = arg \min _{x} [ - \sum_{t} \log P( \mathbf{o}_{t} \mid \mathbf{x}) - \log P(\mathbf{x})]

Assume the observation obey to Gaussian distrubution :

.. math::
    P(\mathbf{o}_{t} \mid \mathbf{x}) = \mathcal{N} (\mu_{t}, \Sigma_{t}) , P(\mathbf{x}) = \mathcal{N} (\mu_{x}, \Sigma_{x})

.. math::
    \bar{\mathbf{x}_{MAP}} = arg \min _{x} \sum_{t} \| \mathbf{o}_{t} - \mu_{t} \|_{\Sigma_{t}}^{2} + \| \mathbf{x} - \mu_{x}  \|_{\Sigma_{x}}^{2}

Which is non-linear least squares problem.




What is it?
------------------

Two basic questions:
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 * where am I?
 * What is the environment around me?

Sensor
>>>>>>>>>>>>>>>>>>>>>>>>>>>
.. image:: images/sensors.PNG
   :scale: 80 %
   :align: center
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


SLAM structure
>>>>>>>>>>>>>>>>>>>>>
* front end : Visual Odometry (use close frames to calculate poses, feature points based methods or direct methods)
* back end : Optimization (Filter based methods, graph optimization methods)
* Loop Closing : Detection and Correction
* Mapping : Make the map

Math description
>>>>>>>>>>>>>>>>>>>>

（to do）


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
