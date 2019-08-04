Paper read
=========================

Gauge Freedom
-----------------------

文章 [#]_ 主要探讨了 **Gauge Freedom** 对VI系统的影响。Gauge Freedom可以理解为系统的不确定性，不能观测的变量，或者Hessian矩阵的零空间大小。对于纯视觉SLAM来说，维度是 **7** ，包含了六个初始位姿的不确定和尺度scale的不确定性。对VI视觉+IMU系统来说，由于引入了重力的大小和方向，系统的Gauge freedom为 **4** 维，包括三个平移不确定和yaw角度不确定。


比较/分析手段
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

为了稳定的求解VISLAM的系统，需要对这4个自由度做处理。在文章中，分别讨论了三种处理方式，并且分别做了数值分析和讨论。

1. **fixing** the unobservable states to some given values. 固定这些不能观测的量，具体的来说就是固定初始的位姿 :math:`p_{0}^{0}, R_{0}^{0}` 。
2. setting a **prior** on such states. 给这些不能观测量加入先验知识。具体来说，给初始位姿对应的Hessian矩阵通过叠加对角阵的形式增加先验信息。
3. letting the states evolve **freely** . 什么都不做，任其发展。

Prior方式的处理相当于另外两种的平均。和Fixation方式一样，它也是给不可观测量增加约束，但是同时加入了状态的不确定性。所以：

* 如果叠加的对角阵的元素值非常大，结果会和Fixation gauge基本一致。
* 如果叠加的对角阵元素非常小，接近零，结果则会和Free gauge的方式一致。

在分析方面，对三个方向进行分析：

1. accuracy. 精度方面，先将结果的trajectory统一align。之后，位置的误差通过关键帧的物理距离衡量，旋转的误差，通过相对角度的大小衡量。
2. computational cost. 运算时间，通过模拟50次优化，分别衡量总时间、迭代次数、每次迭代的时间。
3. estimated covariance. 结果的方差。由于他们在分形空间中不统一，作者采取了线性转换的形式将他们在高维的参数空间中统一，之后分析。

VI系统的 **残差** residual包含了IMU项和视觉项，分别使用和普遍VIO相同的模型模拟：视觉使用重投影模型、IMU使用预测和测量的差。

**模拟数据** 使用了平面点和随机点的组合，另外初始位置也加入一定的扰动，使用 B splines 模拟IMU数据。另外也使用了EuRoC的 **真实数据** 进行了比较。

问题阐述
~~~~~~~~~~~~~~~~~~~~~~~~~~~
在VISLAM中，由于缺少了整体的平移和yaw角度的观测，系统是有多组解的（没有唯一解）。那么多解可以描述为：**参数空间中的流型（~特殊的高维几何）** ，在这个流型 **M** 上的所有点都是系统的解。为了得到唯一解，可以选择在空间中增加约束，约束会通过另一个高维几何体 **C** 的形式体现，而约束之后的解一定会落在 **M** 和 **C** 的交集（参数空间的几何上的交界处）上。

.. image:: images/maniford.PNG
   :width: 50%
   :align: center
   
如上图所示，Fixation gauge方式的结果会在 **M** 和 **C** 的交界处，Free gauge方式的结果会落在 **M** 上（具体位置会收到Start初值影响）。根据前面的分析，Prior gauge方式的结果则会落在另外两种方式之间，具体位置由叠加的先验信息矩阵决定。

Prior Gauge的处理
~~~~~~~~~~~~~~~~~~~~~


协方差处理
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

从上图中可以发现，其实几种方法得到的协方差矩阵其实是不统一的，没办法直接做比较。
从直接的协方差矩阵结果可以看出：

* Free gauge的协方差平均的“分布”在所有的变量上。
* 由于增加了约束，Fixation gauge的第一个位姿的协方差为零（由于它被固定了），之后位姿的协方差会不停的增加（由于误差是会不停累加的，很符合物理现实）。

所以作者对Free Gauge的协方差结果做了如下的变换。

.. image:: images/manifordTranform.PNG
   :width: 50%
   :align: center

1. 将Free Gauge的结果 :math:`\theta` 和 :math:`\Delta \theta` 在 **M** 流型上线性平移到与 **C** 相交的位置。
2. 在这个位置，对 :math:`\theta` 和 :math:`\Delta \theta` 在 **C** 的切平面上分解，并取出切方向的分量。
3. 计算新的对应的协方差矩阵，由同时线性平移的 :math:`\Delta \theta` 求出（具体表达式详见原文章）。

.. image:: images/transformed.PNG
   :width: 100%
   :align: center

线性平移变换之后的Free gauge的协方差矩阵和Fixation gauge的协方差矩阵结果基本是一致的。对于作者的模拟数据集，差距的为
0.11%，而EuRoC的结果差距为0.02%。可以认为，两个系统的协方差是一致的。


实验结果
~~~~~~~~~~~~~~~~~~~~~~~~


.. image:: images/eurocResult.PNG
   :width: 70%
   :align: center

.. [#] Zhang Z, Gallego G, Scaramuzza D. On the comparison of gauge freedom handling in optimization-based visual-inertial state estimation[J]. IEEE Robotics and Automation Letters, 2018, 3(3): 2710-2717.
