PGM
===========================
Probabilistic graphical model is a method combining probabilistic mothods with graph model for solving problem. It is consist of:

1. Build the model.
2. Propagate the probability.
3. Learn the parameters (solve the problem).

It can be solved via a Maximum a posteriori (MAP) inference of a Bayseian inference. Only when having a gaussian assumpution they will obtain the same result. Here, we consider the Maximum a posteriori (MAP) inference which is to optimize the energy function E (which has been proven to be equal to Brief propagation algorithm [1]_ ).

Here we want to solve image segmentation task with energy minimization using PGM theory.



Reference
-----------------

.. [1] Yedidia J S, Freeman W T, Weiss Y. Constructing free-energy approximations and generalized belief propagation algorithms[J]. IEEE Transactions on information theory, 2005, 51(7): 2282-2312.
