Cost Functions
==============================

Cost functions for optimization

Useful equations :

.. math::
  \mathrm{Exp}(\phi + \delta \phi) \approx \mathrm{Exp}(\phi)\mathrm{Exp}(J_{r}(\phi)\delta \phi)

.. math::
  \mathrm{Log}(\mathrm{Exp}(\phi)\mathrm{Exp}(\delta \phi)) \approx \phi + J_{r}^{-1}(\phi)\delta \phi

.. math::
  \begin{align}
  &R \mathrm{Exp}(\phi)R^{T} = exp(R[\phi]_{\times}R^{T}) = \mathrm{Exp}(R\phi) \\
  &\Leftrightarrow \quad \mathrm{Exp}(\phi)R = R \mathrm{Exp}(R^{T}\phi)
  \end{align}

.. toctree::
   :maxdepth: 3
   :caption: Contents:

   RelativePose
   IMU
   VisualMapPoint
