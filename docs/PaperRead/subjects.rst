Specific Subjects
==================

ICP covariance
---------------

**ICP error source**:

* wrong convergence (to local minimial), error of the initial pose estimation.
* under-constrainted situation: the problem is indeterminted.
* miss match.
* sensor noise.

|chrown|  `An accurate closed-form estimate of ICP's covariance 2007 <https://ieeexplore.ieee.org/document/4209579>`_.
Use hessien matrix as the estimation of the covariance:

.. math::
  cov(\hat{x}) \approx 2\frac{residual}{#matches-3} [\frac{\partial^{2}}{\partial x^{2}}residual]^{-1}

And more generally : 

.. math::
  cov(x) \approx (\frac{\partial r^{2}}{\partial x^{2}})^{-1} (\frac{\partial r^{2}}{\partial z\partial x) cov(z) 
  (\frac{\partial r^{2}}{\partial z\partial x)^{T} (\frac{\partial r^{2}}{\partial x^{2}})^{-1}

|thumbs| `A Closed-form Estimate of 3D ICP Covariance 2015 <https://sites.google.com/site/icpcovariance/>`_.
Based on the upper paper, and solve for point-to-point case.

|thumbs| `On the Covariance of ICP-based Scan-matching Techniques 2016 <https://arxiv.org/abs/1410.7632>`_.
Analysis the upper hessien based method. Find that the upper method fit for point-to-plane icp, but not for point-to-point icp.


|thumbs| `A New Approach to 3D ICP Covariance Estimation 2019 <https://arxiv.org/abs/1909.05722>`_.
Add an additional term for the covariance from the initial pose estimation.

.. |chrown| image:: images/chrown.png
    :width: 3%

.. |chrown0| image:: images/chrown0.png
    :width: 3%

.. |thumbs| image:: images/thumbs.png
    :width: 3%

.. |unhappy| image:: images/unhappy.png
    :width: 3%

.. |question| image:: images/question.png
    :width: 3%
