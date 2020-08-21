14. Influence Maximization
=============================

**Viral Marketing** :  a strategy that uses existing social networks to spread and promote a product.
A well-engineered viral marking compaign will identify the most influential customers, convince them
to adopt and endorse the product, and then spread the product in the social network like a virus.

-> find the most influential set of nodes.

14.1 Linear Threshold Model
------------------------

Each node influenced linearly by its neighbors. And get activated if supress its threshold.

* A node has a random threshold :math:`\theta_{v} \sim U[0,1]`.
* A node v influenced by each neighbor w according to a weight :math:`b_{v,w}`, such that :

.. math::
  \sum_{w\in N(v)}b_{v,w} \le 1

* A node v becomes active when at least :math:`\theta_{v}` fraction of its neighbors are active. That is :

.. math::
  \sum_{w\in N(v)\ active} b_{v,w} \ge \theta_{v}

14.2 Independent Cascade Model
---------------------------

If node v is active, it gets **one** chance to make w active, with probability :math:`p_{vw}`.

* Each edge fires only once.
* If u and v are both active and link to w, it does not matter which tries to activate w first.


14.3 Influential Maximization (of ICM)
------------------------------

**Most influential Set of size k** , (k is a user-defined parameter) is a set S
containing k nodes that if activated, produces the largest expected cascade size f(S).
optimzation problem (NP hard, at least as hard as a set cover problem) :

.. math::
  \max_{\# S = k}f(S) = \max_{\# S = k}\farc{1}{\mid I\mid}\sum_{i \in I\ random\ simulation}f_{i}(S)

**Influence set** :math:`X_{u}` of node u, is the set of nodes that will be eventually (expected)
activated by node u.

.. math::
  f(S) = \mid \cup_{u\in S}X_{u} \mid

**Hill Climbing** that gives a sub-optimal solution (:math:`f(S)\ge (1-1/e)f(S_{opt})`) [1]_ :
At each iteration, activate the node u that gives the largest marginal gain:

.. math::
  \max_{u}f(S_{i-1}\cap \{ u\})

.. [1] As it is monotone and submodular.
