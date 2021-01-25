Relative Pose
====================

Skip the translation parts, which are simple.
Note only rotation parts (using right pertubration model).

Residual
------------------

.. math::
  r_{rotation} = \mathrm{Log}(R_{1,2}\mathrm{Exp}(-\phi_{w,2})\mathrm{Exp}(\phi_{w,1})) = \bar{\phi}


Jacobians
------------------

Jacobians w.r.t :math:`\phi_{w,1}`

.. math::
  \begin{align}
  \frac{\partial r}{\partial \phi_{w,1}} &= \frac{\partial \mathrm{Log}(R_{1,2}\mathrm{Exp}(-\phi_{w,2})\mathrm{Exp}(\phi_{w,1}))}
  {\partial \phi_{w,1}}\\
  &= \lim_{\delta \phi \to 0}\frac{\mathrm{Log}(R_{1,2}\mathrm{Exp}(-\phi_{w,2})\mathrm{Exp}(\phi_{w,1})\mathrm{Exp}(\delta\phi))-
  \mathrm{Log}(R_{1,2}\mathrm{Exp}(-\phi_{w,2})\mathrm{Exp}(\phi_{w,1}))}{\delta \phi} \\
  &= \lim_{\delta \phi \to 0}\frac{\mathrm{Log}(\mathrm{Exp}(\bar{\phi})\mathrm{Exp}(\delta\phi))-\bar{\phi}}{\delta \phi} \\
  &= \lim_{\delta \phi \to 0}\frac{\bar{\phi}+J_{r}^{-1}(\bar{\phi})\delta\phi -\bar{\phi}}{\delta \phi} \\
  &= J_{r}^{-1}(\bar{\phi})
  \end{align}

Jacobians w.r.t :math:`\phi_{w,2}`

.. math::
  \begin{align}
  \frac{\partial r}{\partial \phi_{w,2}} &= \frac{\partial \mathrm{Log}(R_{1,2}\mathrm{Exp}(-\phi_{w,2})\mathrm{Exp}(\phi_{w,1}))}
  {\partial \phi_{w,2}}\\
  &= \lim_{\delta \phi \to 0}\frac{\mathrm{Log}(R_{1,2}\mathrm{Exp}(-(\phi_{w,2}+\delta\phi))\mathrm{Exp}(\phi_{w,1}))-
  \bar{\phi}}{\delta \phi} \\
  &= \lim_{\delta \phi \to 0}\frac{\mathrm{Log}(R_{1,2}\mathrm{Exp}(-\delta\phi)\mathrm{Exp}(-\phi_{w,2})\mathrm{Exp}(\phi_{w,1}))-
  \bar{\phi}}{\delta \phi} \\
  &= \lim_{\delta \phi \to 0}\frac{\mathrm{Log}((\mathrm{Exp}(\delta\phi)R_{1,2}^{-1})^{-1}\mathrm{Exp}(-\phi_{w,2})\mathrm{Exp}(\phi_{w,1}))-
  \bar{\phi}}{\delta \phi} \\
  &= \lim_{\delta \phi \to 0}\frac{\mathrm{Log}((R_{1,2}^{-1}\mathrm{Exp}(R_{1,2}\delta\phi))^{-1}\mathrm{Exp}(-\phi_{w,2})\mathrm{Exp}(\phi_{w,1}))-
  \bar{\phi}}{\delta \phi} \\
  &= \lim_{\delta \phi \to 0}\frac{\mathrm{Log}(\mathrm{Exp}(-R_{1,2}\delta\phi)\mathrm{Exp}(\bar{\phi}))-
  \bar{\phi}}{\delta \phi} \\
  &= \lim_{\delta \phi \to 0}\frac{\mathrm{Log}(\mathrm{Exp}(\bar{\phi})\mathrm{Exp}(-\mathrm{Exp}(-\bar{\phi})R_{1,2}\delta\phi))-
  \bar{\phi}}{\delta \phi} \\
  &= \lim_{\delta \phi \to 0}\frac{\bar{\phi} + J_{r}^{-1}(\bar{\phi})(-\mathrm{Exp}(-\bar{\phi})R_{1,2}\delta\phi)-
  \bar{\phi}}{\delta \phi} \\
  &= - J_{r}^{-1}(\bar{\phi})\mathrm{Exp}(-\bar{\phi})R_{1,2}
  \end{align}
