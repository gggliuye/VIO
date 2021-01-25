IMU
===================

Ordinary integration between two imu data, (not pre-integrator).

Variables for i-th state :math:`[p_{w,b_{i}}, r_{w,b_{i}}, v_{i}^{w}, b_{i}^{a}, b_{i}^{g}]^{T}` .

For simplicity, keep bias constant (or linear intepolation from start to end). As in my usage,
we will have a optimized starting state and ending state, and interval time not too long.

.. math::
  a_{measurement} = q_{w, b}(a_{real}-g)+b+n

Residual
----------------

The hat value could be unbiased measurement or mid point value.

.. math::
  \begin{bmatrix} r_{p} \\  r_{q} \\ r_{v} \end{bmatrix} =
  \begin{bmatrix} p_{w,b_{i}} - p_{w,b_{j}} + v_{i}^{w}\delta t + \frac{1}{2} g \delta t^{2} + \frac{1}{2}\mathrm{Exp}(r_{w,b_{i}})\hat{a}\delta t^{2} \\
  \mathrm{Log}(\mathrm{Exp}(-r_{w,b_{j}}) \mathrm{Exp}(r_{w,b_{i}}) \mathrm{Exp}(\hat{w}\delta t) ) \\
  v_{i}^{w} - v_{j}^{w} + \mathrm{Exp}(r_{w,b_{i}})\hat{a}\delta t + g\delta t
  \end{bmatrix}


Jacobians
-----------------
Jacobians of position :

.. math::
  \frac{\partial r_{p} }{\partial p_{w, b_{i}}} = I_{3\times 3}

.. math::
  \begin{align}
  \frac{\partial r_{p} }{\partial r_{w, b_{i}}} &= \frac{\delta t^{2}}{2}\frac{\partial \mathrm{Exp}(r_{w,b_{i}})\hat{a} }{\partial r_{w, b_{i}}} \\
  &= \frac{\delta t^{2}}{2}\lim_{\delta r\to 0}\frac{\mathrm{Exp}(r_{w,b_{i}})\mathrm{Exp}(\delta r)\hat{a} -\mathrm{Exp}(r_{w,b_{i}})\hat{a} }{\delta r} \\
  &= \frac{\delta t^{2}}{2}\lim_{\delta r\to 0}\frac{\mathrm{Exp}(r_{w,b_{i}})[\delta r]_{\times}\hat{a} }{\delta r} \\
  &= -\frac{\delta t^{2}}{2}\mathrm{Exp}(r_{w,b_{i}})[\hat{a}]_{\times}
  \end{align}

.. math::
  \frac{\partial r_{p} }{\partial v_{i}^{w}} = \delta t I_{3\times 3}

.. math::
  \frac{\partial r_{p} }{\partial p_{w, b_{j}}} = -I_{3\times 3}

.. math::
  \frac{\partial r_{p} }{\partial r_{w, b_{j}}} = 0

.. math::
  \frac{\partial r_{p} }{\partial v_{j}^{w}} = 0


Jacobians of rotation :

.. math::
  \bar{\phi} =  \mathrm{Log}(\mathrm{Exp}(-r_{w,b_{j}}) \mathrm{Exp}(r_{w,b_{i}}) \mathrm{Exp}(\hat{w}\delta t) )

.. math::
  \begin{align}
  \frac{\partial r_{q} }{\partial r_{w, b_{i}}}
  &= \frac{\partial \mathrm{Log}(\mathrm{Exp}(-r_{w,b_{j}}) \mathrm{Exp}(r_{w,b_{i}}) \mathrm{Exp}(\hat{w}\delta t) )}
  {\partial r_{w, b_{i}}} \\
  &= \lim_{\delta r\to 0}\frac{ \mathrm{Log}(\mathrm{Exp}(-r_{w,b_{j}}) \mathrm{Exp}(r_{w,b_{i}})
  \mathrm{Exp}(\delta r)\mathrm{Exp}(\hat{w}\delta t)) - \bar{\phi}}{\delta r} \\
  &= \lim_{\delta r\to 0}\frac{ \mathrm{Log}(\mathrm{Exp}(-r_{w,b_{j}}) \mathrm{Exp}(r_{w,b_{i}})
  \mathrm{Exp}(\hat{w}\delta t)\mathrm{Exp}(\mathrm{Exp}(-\hat{w}\delta t) \delta r)) - \bar{\phi}}{\delta r} \\
  &= \lim_{\delta r\to 0}\frac{J_{r}^{-1}(\bar{\phi})\mathrm{Exp}(-\hat{w}\delta t) \delta r}{\delta r} \\
  &= J_{r}^{-1}(\bar{\phi})\mathrm{Exp}(-\hat{w}\delta t)
  \end{align}

.. math::
  \begin{align}
  \frac{\partial r_{q} }{\partial r_{w, b_{j}}}
  &= \frac{\partial \mathrm{Log}(\mathrm{Exp}(-r_{w,b_{j}}) \mathrm{Exp}(r_{w,b_{i}}) \mathrm{Exp}(\hat{w}\delta t) )}
  {\partial r_{w, b_{j}}} \\
  &= \lim_{\delta r\to 0}\frac{ \mathrm{Log}(\mathrm{Exp}(-\delta r)\mathrm{Exp}(-r_{w,b_{j}}) \mathrm{Exp}(r_{w,b_{i}})
  \mathrm{Exp}(\hat{w}\delta t)) - \bar{\phi}}{\delta r} \\
  &= \lim_{\delta r\to 0}\frac{ \mathrm{Log}(\mathrm{Exp}(-\delta r)\mathrm{Exp}(\bar{\phi})) - \bar{\phi}}{\delta r} \\
  &= \lim_{\delta r\to 0}\frac{ \mathrm{Log}(\mathrm{Exp}(\bar{\phi})\mathrm{Exp}(-\mathrm{Exp}(-\bar{\phi})\delta r)) - \bar{\phi}}{\delta r} \\
  &= -J_{r}^{-1}(\bar{\phi})\mathrm{Exp}(-\bar{\phi})
  \end{align}

Jacobians of velocity :

.. math::
  \begin{align}
  \frac{\partial r_{v} }{\partial r_{w, b_{i}}}
  &= \frac{\partial \mathrm{Exp}(r_{w,b_{i}})\hat{a}\delta t}{\partial r_{w, b_{i}}} \\
  &= \lim_{\delta r \to 0}\frac{\mathrm{Exp}(r_{w,b_{i}})[\delta r]_{\times}\hat{a}\delta t}{\delta r} \\
  &= -\mathrm{Exp}(r_{w,b_{i}})[\hat{a}]_{\times}\delta t
  \end{align}

.. math::
  \frac{\partial r_{v} }{\partial v_{i}^{w}} = I_{3\times 3}

.. math::
  \frac{\partial r_{v} }{\partial v_{j}^{w}} = -I_{3\times 3}
