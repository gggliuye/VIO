Paper read
=========================

Gauge Freedom
-----------------------

文章主要探讨了 **Gauge Freedom** 对VI系统的影响。Gauge Freedom可以理解为系统的不确定性，不能观测的变量，或者Hessian矩阵的零空间大小。对于纯视觉SLAM来说，维度是 **7** ，包含了六个初始位姿的不确定和尺度scale的不确定性。对VI视觉+IMU系统来说，由于引入了重力的大小和方向，系统的Gauge freedom为 **4** 维，包括三个平移不确定和yaw角度不确定。

为了稳定的求解VISLAM的系统，需要对这4个自由度做处理。在这篇文章中，分别讨论了三种处理方式，并且分别做了数值分析和讨论。

1. fixing the unobservable states to some given values. 固定这些不能观测的量，具体的来说就是固定初始的位姿 :math:`p_{0}^{0}, R_{0}^{0}` 。
2. setting a prior on such states. 给这些不能观测量加入先验知识。具体来说，给初始位姿对应的Hessian矩阵通过叠加对角阵的形式增加先验信息。
3. letting the states evolve freely. 上面都不做，任其发展。







