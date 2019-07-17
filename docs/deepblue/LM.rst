Levenberg-Marquardt Method
=================================


.. math::

    y = c \cdot e^{a \cdot x} + d \cdot e^{b \cdot x}
 
.. math::

    Residual = \sum_{i} (c \cdot e^{a \cdot x_{i}} + d \cdot e^{b \cdot x_{i}} - \overline{y_{i}} )


.. math::

    J = [ x_{i}*c*e^{a*x_{i}}  , x_{i}*d*e^{b*x_{i}}, e^{a*x_{i} , e^{b*x_{i}} ]
   
