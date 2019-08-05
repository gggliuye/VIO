Visual Localization
===================


CVPR
----------------------

**EN:**
The objective of this `CVPR competition <https://www.visuallocalization.net/>`_ is localization in different environment changes (winter/spring, rain/snow/sunday, day/night, etc). The core of these algorithms is their robustness against these environment change. The main warpon is "Deep learning", for example EHTZ use its own HF-net [#]_ to extract features, and CAS uses `Deeplabv3 <https://github.com/rishizek/tensorflow-deeplab-v3>`_ (an open source deep learning architecture ) to extract semantic feature as a criterion for outlier rejection [#]_.


**CH：**
这个竞赛解决的核心问题是针对变化环境的鲁棒性处理(定位的精度并不是它的核心问题)。所以算法的核心在与：使用深度学习工具，构造出鲁棒性更好更加robust的系统。为了这个目的，苏黎世大学的方案是用深度学习提取特征，中国科学院的方式的把深度学习的语义信息用来排除外点。值得一提的是，他们在SFM建模都使用了colmap的开源框架，苏黎世自己创建了深度学习模型，中科院则是运用了开源框架deeplab。

.. image:: bannerCVPR.png
   :align: center
   :alt: image from CVPR main page

1st EHTZ
~~~~~~~~~~~~~~~~~~~~

They use a **Hierarchical Localization** algorithm, using monolihic CNN to simultaneously(sharing of computations) predict **local features** and **global descriptors** for localization.  `github page <https://github.com/ethz-asl/hfnet>`_  `paper <https://arxiv.org/abs/1812.03506>`_

1. Global descriptor is used to retrieves candidate images.
2. Local features are used to esimate an accurate 6-DoF pose.

The robustness of localization is maximized while retaining tractable computational requirements.

**HF-Net**

It is composed of a single encoder (a **MobileNet** bakcbone: optimized for mobile inference) and three heads predicting : 

1. keypoint detection scores
2. dense local descriptors
3. a global image-wide descriptor.

The global descriptor is predicted by NetVLAD layer on top of the last feature map of Moblie Net.     

1st CAS
~~~~~~~~~~~~~~~~~~~~
**EN:** From China Academy of Sciences.
The following image shows the system pipeline. They used Colmap [#]_ for offline SfM reconstruction, DeeplabV3 [#]_ to offer semantic segmentation, and NetVLAD [#]_ to offer image match. 

**CH:** 下图是整个系统的流程图解析，SfM离线建图的部分使用了Colmap，语义分割使用了Deeplabv3，图像匹配使用了NetVLAD。总的来说，排除外点的想法具有原创性。

.. image:: CAS.png
   :align: center


**Contribute：**

**EN：**

* a new localization pipeline : use semantic infomation as outlier rejection criterion.
* do not need any additional restrictions (e.g. camera height, grivaty)

**CH：**

* 一个新的定位模式框架：使用语义信息提供新的排除外点的标准。
* 不需要一些额外信息（比如相机的高度和重力方向）。

**personal view**: lack of originality. 


Ours
----------------------
They all used Colmap pipeline as basic, and used deep learning mehtod to upate the features(add semantic labels / deep learning extraction method). As a result, we choose to go with the same direction : start from colmap structure, and use deep learning to help updating.

Colmap
~~~~~~~~~~~~~~~~~~~~~~~
Colmap offers tool to enable second development. However, I think it is better to use its results only to make it a seperated system. 

1. Decode Colmap's result
2. For new input image, use the same feature extraction method to extract features. 
3. Match with image database.
4. solve pose for the input image.




Reference
-----------------

.. [#] Sarlin P E, Cadena C, Siegwart R, et al. From coarse to fine: Robust hierarchical localization at large scale[C]//Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition. 2019: 12716-12725.

.. [#] Shi T, Shen S, Gao X, et al. Visual Localization Using Sparse Semantic 3D Map[J]. arXiv preprint arXiv:1904.03803, 2019.

.. [#] Schonberger J L, Frahm J M. Structure-from-motion revisited[C]//Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition. 2016: 4104-4113.

.. [#] Chen L C, Papandreou G, Kokkinos I, et al. Deeplab: Semantic image segmentation with deep convolutional nets, atrous convolution, and fully connected crfs[J]. IEEE transactions on pattern analysis and machine intelligence, 2017, 40(4): 834-848.

.. [#] Arandjelovic R, Gronat P, Torii A, et al. NetVLAD: CNN architecture for weakly supervised place recognition[C]//Proceedings of the IEEE conference on computer vision and pattern recognition. 2016: 5297-5307.
