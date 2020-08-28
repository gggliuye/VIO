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
optimzation problem (NP hard [1]_ ) :

.. math::
  \max_{\# S = k}f(S) = \max_{\# S = k}\frac{1}{\mid I\mid}\sum_{i \in I\ random\ simulation}f_{i}(S)

**Influence set** :math:`X_{u}` of node u, is the set of nodes that will be eventually (expected)
activated by node u.

.. math::
  f_{i}(S) = \mid \cup_{u\in S}X_{u}^{i} \mid

**Hill Climbing** that gives a sub-optimal solution (:math:`f(S)\ge (1-1/e)f(S_{opt})`) [2]_ :
At each iteration, activate the node u that gives the largest marginal gain:

.. math::
  \max_{u}f(S_{i-1}\cap \{ u\})

**Monotone** : If S is a subset of T, then :math:`f(S)\le f(T)`, and :math:`f(\emptyset) = 0`.

**Submodular** : If S is a subset of T, then for any node u [3]_ :

.. math::
  f(S\cup \{ u\}) -f(S) \ge f(T\cup \{ u\}) -f(T)

While the greedy approach is slow. **Sketch-based** algorithm to accelerate : compute small structure per node from which to estimate its influence.
Then run influence maximization using these estimates.

.. [1] See the hand-on, we could prove **Set Cover** problem (briefly take k subsets to cover the most), which is NP-hard, could be reduced to Influence maximization problem.

.. [2] As it is monotone and submodular. See the prove in the hand-on.

.. [3] Which means that adding a node to a set has less impact ("marginal gain") than adding the same node to a smaller subset of that set. f is submodular as it is a positive linear combination of submodualr functions :math:`f_{i}` .


15. Outbreak Detection
=========================
