7. Graph Representation Learning
===============================

7.1 Introcution
---------------

**Supervised Learning** :

* Raw Data -> Structural Data -> Learning Algorithm -> Model.
* Main part : *feature engineering*, ML/DL: automatically learn the features.

**Efficient feature learning**: :math:`f: u\to \mathcal{R}^{d}`. *network embedding*.

**Difficulty** : modern DL toolbox is designed for simple sequence or grids (example, chain for NLP,
2D grid for image processing, etc). But real graph is much more complicate and could be dynamic.

7.2 Node Embedding
----------------------

.. math::
  ENC(v) = z_{v} \ \in \mathcal{R}^{d}

In order to realize :

.. math::
  similarity (u,v) \approx z_{v}^{T}z_{u}

* Define an encoder (e.g. *Shallow encoding* :math:'ENC(v) = Zv, \ Z\in \mathcal{R}^{d\times \mid V\mid}').
* Mode simiarity function.
* Optimize the parameters.

7.3 Random Walk
--------------------

Interpretation: :math:`z_{v}^{T}z_{u}` approximate the probability that u and v co-occur in a random walk over the network.
Different random work setup could capture different properties. **It is efficient and expressive**.

**Summary** :

* Run short fixed-length random walks starting from each node on the graph using some strategy R.
* For each node u collect :math:`N_{R}(u)`, the multiset of nodes visited on random walks starting from u.
* Optimize embeddings using Stochastic Gradient Descent (We can efficiently approximate this using negative sampling [1]_ )

.. math::
  \begin{align*}
  \mathcal{L} &= \sum_{u\in V}\sum_{v\in N_{R}(u)} - \log (P(v\mid z_{u})) \\
  & =  \sum_{u\in V}\sum_{v\in N_{R}(u)} -\log(\frac{\exp(z_{v}^{T}z_{u})}{\sum_{n\in V}\exp(z_{n}^{T}z_{u})})
  \end{align*}

.. [1] Noise Contrastive Estimation (NCE), randomly sample the approximate the lower part of the expression of P (softmax fcn), to lower the computaional cost.

7.4 Node2Vec
-------------------

Goal: Embed nodes with similar network neighborhoods close in the feature space, with a more flexible notion of network
neighborhood.
