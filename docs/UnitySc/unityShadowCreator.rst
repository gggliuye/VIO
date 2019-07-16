AR Garden
=========================================

简介
--------------

这个项目的展示效果是“AR花园”，使用影创的AR眼镜设备，在我们的场景中，实现虚拟与现实的结合。同时现实的物体会遮挡虚拟物体，用户也可以实现和虚拟物体的互动。

主要的实现方法是：定期服务器提供绝对位置，影创眼镜设备则负责追踪。
我们第一阶段demo使用的是改良的ORBSLAM2框架作为云端定位的模型，设备的追踪则依靠影创提供了嵌入式VIO。之后通过算法，将两个系统的结果结合，最终得到我们想要的--实时的用户相对于世界坐标系的位姿（位姿：位置+姿态，它有六个自由度=3位移+3旋转）。


系统坐标系
----------------
在整个系统中一共有两个坐标系：世界坐标系（云端定位系统的坐标系统，这也将是Unity3d的坐标系）和本地坐标系（影创眼镜VIO追踪的坐标系），即是下图中的world coordinate system和local coordinate system。

云端定位可以得到一个位姿，由于云端定位系统本质是图像定位，得到的结果是对应相机（camera）光心的位姿。影创的追踪系统（通过两个内置鱼眼相机和较高精度IMU）经过处理之后得到的眼镜环的中心（也就是“头head“）的坐标。
对于Unity的3d虚拟显示，需要两只虚拟眼的位置，所以在Unity系统中需要“头”和“双眼”的相对位置外参（transpose head-eye）。对于定位系统，由于需要统一两个坐标位姿，所以需要“头”和“相机”的相对位置外参（transpose head-camera）。

.. image:: pic1.png
   :width: 80%
   :align: center

原始结果
----------------------------
我们可以分别得到云端定位和本地追踪的两个原始坐标。
一个是P（global pose），即云端定位的结果，是相机光心的位姿；以及P（local pose），即本地追踪的结果，是AR相机虚拟“头”的位姿。
* 这里要注意我们直接从ORBSLAM和影创SDK VIO中得到的原始结果都是相机在对应参考系的坐标，他们是对应变换矩阵的逆。
也就是：
.. math::
    P_{local pose} = T_{local to camera}^{-1}
    
    P_{global pose} = T_{global to camera}^{-1}

位置融合
--------------------

* 我们的目的可以总结为：求两个坐标系（world和local）之间的相对位姿（下图中的黄色变换T transpose local to global）。
* 值得一提的是，由于影创SDK是VIO系统，所以误差会不停的累积，这就导致上面需要求的相对位姿其实不是一个固定的值，而是会随着本地追踪的误差而改变的值。这就要求我们定期的更新这个相对坐标，以保证系统长时间有效。
* 一旦求得了T（transpose local to global），在结合实时得到的P（local pose）我们就可以得到我们想要的用户“头”head，在世界坐标系中的位姿P（Objective pose）。
* 另外，这里描述的所有位姿或者变换，除了P（local pose real time）以外都不要求实时性。

.. image:: pic2.png
   :width: 80%
   :align: center

由上图和分析，我们可以得到下面的表达式
.. math::
    P_{real local pose} = P_{local pose} * T_{camera to head}

    T_{local to global} = P_{real local pose} * P_{global pose}^{-1}

    P_{Objective pose} = T_{local to global} * P_{local pose real time}

结合上面三式，我们有：
.. math::
    P_{Objective pose} = P_{local pose} * T_{camera to head} * P_{global pose}^{-1} * P_{local pose real time}
