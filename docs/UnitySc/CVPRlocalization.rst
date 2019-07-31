CVPR
===================

` Visual Localization <https://www.visuallocalization.net/workshop/cvpr/2019/>`_ 

**EN：**
The objective of this competition is localization in different environment changes (winter/spring, rain/snow/sunday, day/night, etc). The core of these algorithms is their robustness against these environment change. The main warpon is "Deep learning", for example EHTZ use its own HF-net [#]_ to extract features, and CAS uses `Deeplabv3 <https://github.com/rishizek/tensorflow-deeplab-v3>`_ (an open source deep learning architecture ) to extract semantic feature as a criterion for outlier rejection [#]_.


**CH：**
这个竞赛解决的核心问题是针对变化环境的鲁棒性处理(定位的精度并不是它的核心问题)。所以算法的核心在与：使用深度学习工具，构造出鲁棒性更好更加robust的系统。为了这个目的，苏黎世大学的方案是用深度学习提取特征[1]_ ，中国科学院的方式的把深度学习的语义信息用来排除外点[2]_ 。值得一提的是，他们在SFM建模都使用了colmap的开源框架，苏黎世自己创建了深度学习模型，中科院则是运用了开源框架deeplab。

.. image:: bannerCVPR.png
   :align: center

1st EHTZ
--------------------

2nd CAS
------------------------
China Academy of Sciences.

contribute
~~~~~~~~~~~~~~~~

**EN：**

* a new localization pipeline : use semantic infomation as outlier rejection criterion.
* do not need any additional restrictions (e.g. camera height, grivaty)

**CH：**

* 一个新的定位模式框架：使用语义信息提供新的排除外点的标准。
* 不需要一些额外信息（比如相机的高度和重力方向）。

**personal view**: lack of originality. 



.. [#] Sarlin P E, Cadena C, Siegwart R, et al. From coarse to fine: Robust hierarchical localization at large scale[C]//Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition. 2019: 12716-12725.

.. [#] Shi T, Shen S, Gao X, et al. Visual Localization Using Sparse Semantic 3D Map[J]. arXiv preprint arXiv:1904.03803, 2019.
