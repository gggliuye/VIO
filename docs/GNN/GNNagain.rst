18. Limitations of GNN
============================

Recap `GNN <https://vio.readthedocs.io/zh_CN/latest/GNN/7GraphRepresentation.html#graph-neural-networks>`_ :
**Aggregate Neighbors** **Computation Graph** **Graph Pooling**.

Many model variants have been proposed with different choice of neural networks.

* GCN [Kipf & Welling ICLR'2017]
* GraphSAGE [Hamilton+ NeurIPS’2017]

Applications:

* Node classification [Kipf+ ICLR’2017]
* Graph Classification [Ying+ NeurIPS’2018]
* Link Prediction [Zhang+ NeurIPS’2018]

18.1 Limitation : Graph Structure
----------------------------

**Graph Structure** Some simple graph structures cannot be distinguished by conventional GNNs.
-> **graph isomorphism test problem** .

.. image:: images/limit_1.PNG
  :align: center
  :width: 80%

Node representation captures rooted subtree structure : Most discriminative GNNs map different subtrees
into different node representations.
-> **Injectivity** [1]_ of the funtion to map to different graph structure.
-> **Injective Neighbor Aggregation** . Entire neighbor aggregation is injective if
every step of neighbor aggregation is injective.

**Multi-set Functions** : Neighbor aggregation is essentially a function over multi-set (set with repeating elements).
Discriminative Power of GNNs can be characterized by that of **multi-set functions** .

* GCN (mean pooling) will fail to distinguish proportionally equivalent multi-sets.
* GraphSAGE (max pooling) will even fail to distinguish multi-set with the same distinct elements.

**Injective Multi-set Functions** Theorem: Any injective multi-set function can be expressed by, where  𝜙 and 𝑓 are non-linear functions  :

.. math::
  \phi(\sum_{x\in S}f(x))

We can model 𝜙 and 𝑓 using Multi-Layer-Perceptron (MLP) (Note: MLP is a universal approximator).

.. image:: images/mlp.PNG
  :align: center
  :width: 60%

Graph Isomorphism Network (GIN) [Xu+ ICLR’2019], using MLP + sum pooling.
So far: GIN achieves maximal discriminative power by using injective neighbor aggregation.
GIN fits training data much better than GCN, GraphSAGE.
GIN is closely related to Weisfeiler-Lehman (WL) [2]_ Graph Isomorphism Test (1968).
-> GIN is as discriminative as the WL test.

.. [1] Function is injective if it maps different elements into different outputs.

.. [2] WL test has been known to distinguish most of the graphs, except for some corner cases.

18.2 Limitation : Graph Noise
--------------------

**Graph Noise** GNNs are not robust to noise in graph data.

.. image:: images/limit_2.PNG
  :align: center
  :width: 100%

Adversaries are very common in applications of graph neural networks, e.g., search engines,
recommender systems, social networks, etc.

**Semi-supervised node classification** using Graph Convolutional Neural Networks (GCN) [Kipf+ ICLR’2017]

.. image:: images/attack.PNG
  :align: center
  :width: 80%

**Net Attack** : Zügner+, Adversarial Attacks on Neural Networks for Graph Data, KDD’18.
*Maximize (Change of predicted labels of target node) Subject to (Limited noise in the graph)* .

.. image:: images/attack_math.PNG
  :align: center
  :width: 60%

* Some heuristics have been proposed to efficiently obtain an approximate solution (see the original paper).
* The GCN prediction is easily manipulated by only 5 modifications of graph structure (|V|=~2k, |E|=~5k)
* GCN does neighbor aggregation : only local attack will work.

**GNNs are not robust to adversarial attacks!**

18.2 Future Directions
------------------------

Science Domains:

* Chemistry: Molecular graphs.
* Biology: Protein-Protein Interaction Networks.

Challenges of Applying GNN:

* Scarcity of labeled data : Labels require expensive experiments -> Models overfit to small training datasets.
* Out-of-distribution prediction : Test examples are very different from training in scientific discovery -> Models typically perform poorly.

Pre-training GNNs [Hu+ 2019] : Pre-train GNNs on relevant, easy to obtain graph data.

19. Applications of GNN
==========================

19.1 GNN recommendation (PinSage)
----------------------


19.2 Heterogeneous GNN (Decagon)
---------------------------

19.3 Goal-directed generation (GCPN)
-------------------------------
