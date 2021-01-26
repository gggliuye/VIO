Visual Map Point
====================
Reprojection error of map points.

Use parameterization right perturbation in SE(3), #local variables : 6, # global variables : 6)

.. math::
  \xi \oplus \delta \xi = Log(Exp(\xi)Exp(\delta \xi))

Residual
--------------

.. math::
  e_{k} = \frac{1}{z_{c}}e^{[\xi_{cb}]_{\times}}e^{[\xi_{bw}]_{\times}}p_{w, k} - u_{k}

Where :

* :math:`\xi_{cb}` is the camera-base extrinsic parameters: transformation from base reference frame to camera reference frame.
* :math:`\xi_{bw}` is the base parameters : transformation from world reference frame to base reference frame.
* :math:`p_{w}` is the pose of the 3d map point in the world reference frame.
* z is the depth of the map point in camera reference frame (to transform to camera uv space).
* u is the feature observation (in camera uv space).

.. math::
  p_{b} = e^{[\xi_{bw}]_{\times}}p_{w}

.. math::
  p_{c} = e^{[\xi_{cb}]_{\times}}p_{b}

Jacobians
--------------

Jacobians w.r.t. map points :

.. math::
  J_{p_{w}} = \frac{\partial e}{\partial p_{w}} = \frac{\partial e}{\partial p_{c}} \frac{\partial p_{c}}{\partial p_{w}}

.. math::
  e = \begin{bmatrix} x_{c}/z_{c} \\ y_{c}/z_{c}  \end{bmatrix}
  -\begin{bmatrix} u \\  v \end{bmatrix} = \begin{bmatrix} e_{1} \\  e_{2} \end{bmatrix}

.. math::
  \frac{\partial e}{\partial p_{c}} = \begin{bmatrix} \frac{\partial e_{1}}{\partial x_{c}} &  \frac{\partial e_{1}}{\partial y_{c}} &  \frac{\partial e_{1}}{\partial z_{c}} \\  \frac{\partial e_{2}}{\partial x_{c}} &  \frac{\partial e_{2}}{\partial y_{c}} &  \frac{\partial e_{2}}{\partial z_{c}} \end{bmatrix} =
  \begin{bmatrix} 1/z_{c} & 0 & -x_{c}/z_{c}^{2} \\
  0 & 1/z_{c} & - y_{c}/z_{c}^{2} \end{bmatrix}

.. math::
  \frac{\partial p_{c}}{\partial p_{w}} = \frac{\partial [R_{cb}(R_{bw}p_{w} + t_{bw})+t_{cb}] } {\partial p_{w}}
  = R_{cb}R_{bw}

.. math::
  J_{p_{w}} = \begin{bmatrix} 1/z_{c} & 0 & -x_{c}/z_{c}^{2} \\
  0 & 1/z_{c} & - y_{c}/z_{c}^{2} \end{bmatrix} R_{cb}R_{bw}

Jacobians w.r.t. extrinsic parameters :

.. math::
  J_{p_{w}} = \frac{\partial e}{\partial \xi_{cb}} = \frac{\partial e}{\partial p_{c}} \frac{\partial p_{c}}{\partial \xi_{cb}}

.. math::
  \frac{\partial p_{c}}{\partial \xi_{cb}} = \frac{\partial e^{[\xi_{cb}]_{\times}}p_{b}}{\partial \xi_{cb}} = \frac{\partial}{\partial \xi_{cb}}( R_{cb}p_{b} + t_{cb} ) = \frac{\partial}{\partial \xi_{cb}}( e^{[q_{cb}]_{\times}}p_{b} + t_{cb} )

.. math::
  \begin{align}
  \frac{\partial p_{c}}{\partial \xi_{bc}} &=  \frac{\partial}{\partial \xi_{bc}}( e^{[\xi_{cb}]_{\times}}p_{b} ) \\
  &= \lim_{\delta \xi_{bc}\to 0} \frac{(e^{[\xi_{bc}]_{\times}}e^{[\delta \xi_{bc}]_{\times}})^{-1}p_{b} - e^{[\xi_{cb}]_{\times}}p_{b} }{\delta \xi_{bc}} \\
  &= \lim_{\delta \xi_{bc}\to 0} \frac{e^{[-\delta \xi_{bc}]_{\times}}e^{[\xi_{cb}]_{\times}}p_{b}  - e^{[\xi_{cb}]_{\times}}p_{b} }{\delta \xi_{bc}} \\
  &= \lim_{\delta \xi_{bc}\to 0} \frac{(1-[\delta \xi_{bc}]_{\times})e^{[\xi_{cb}]_{\times}}p_{b} - e^{[\xi_{cb}]_{\times}}p_{b}}{\delta \xi_{bc}} \\
  &= \lim_{\delta \xi_{bc}\to 0} \frac{-[\delta \xi_{bc}]_{\times}e^{[\xi_{cb}]_{\times}}p_{b} }{\delta \xi_{bc}} \\
  &= \lim_{\delta \xi_{bc}\to 0} \frac{-[\delta \xi_{bc}]_{\times}p_{c} }{\delta \xi_{bc}} \\
  &= \lim_{\delta \xi_{bc}\to 0} -\frac{1}{\delta \xi_{bc}}
  \begin{bmatrix}
  [\delta \phi_{bc}]_{x} & \delta t_{bc} \\
  0^{T} & 0
  \end{bmatrix}
  \begin{bmatrix}
  p_{c} \\1
  \end{bmatrix}
  \end{align}

.. math::
  \begin{align}
  \frac{\partial p_{c}}{\partial t_{bc}}
  &=  \lim_{\delta \xi_{cb}\to 0} \frac{-\delta t_{bc}}{\delta t_{bc}}  \\
  &= - I_{3\times 3}
  \end{align}

.. math::
  \begin{align}
  \frac{\partial p_{c}}{\partial \phi_{bc}}
  &= \lim_{\delta \phi_{bc}\to 0} -\frac{1}{\delta \phi_{bc}}
  \begin{bmatrix}
  [\delta \phi_{bc}]_{x} & \delta t_{bc} \\
  0^{T} & 0
  \end{bmatrix}
  \begin{bmatrix}
  p_{c} \\1
  \end{bmatrix} \\
  &=\lim_{\delta \phi_{bc}\to 0} - \frac{[\delta \phi_{bc}]_{\times}p_{c}} {\delta \phi_{bc}} \\
  &=\lim_{\delta \phi_{bc}\to 0} \frac{[p_{c}]_{\times}\delta \phi_{bc}} {\delta \phi_{bc}} \\
  &=[p_{c}]_{\times}
  \end{align}

Jacobians w.r.t. pose base:

.. math::
  \frac{\partial p_{c}}{\partial \xi_{wb}} =  e^{[\xi_{cb}]_{\times}}\frac{\partial}{\partial \xi_{wb}}( e^{[\xi_{bw}]_{\times}}p_{w} )

.. math::
  \frac{\partial p_{c}}{\partial t_{wb}}
  =  -R_{cb}

.. math::
  \frac{\partial p_{c}}{\partial \phi_{wb}}
  =R_{cb}[p_{b}]_{\times}
