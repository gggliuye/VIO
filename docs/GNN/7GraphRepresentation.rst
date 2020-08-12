7. Graph Representation Learning
===============================

This chapter focus the **Node Emedding** based on Machine Learning methods.

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

* Define an encoder (e.g. *Shallow encoding* :math:'ENC(v) = Zv, \ Z\in \mathcal{R}^{d\times \mid V\mid}'). e.g., DeepWalk, node2vec, TransE.
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

.. [1] Computational expensive, as the denominator is intractable. Method : Noise Contrastive Estimation (NCE), randomly sample the approximate the lower part of the expression of P (softmax fcn), to lower the computaional cost.

7.4 Node2Vec
-------------------

Goal: Embed nodes with similar network neighborhoods close in the feature space, with a more flexible notion of network
neighborhood. 

Method : Different random walk parameters for local and global obejctive (parameter p: 1/p probability for return last node, for local property; and parameter q : 1/q 
probability for moving outwards, for global property). Choose parameters p and q could generate specified random walks.

The node2vec algorithm:

* Compute random walk probabilities.
* Simulate r random walks of length l starting from each node u.
* Optimize the node2vec objective using Stochastic Gradient Descent

Applications :

* Clustering using the embedded feature vectors.
* Calculate :math:`f(z_{i})` for node classifications.
* Calculate :math:`f(z_{i}, z_{j})` for link predictions.
 
 7.5 TransE
 --------------------
 
 **Represent the relationships as a linear translation in the embedding space** : :math:`h+l\approx t` , head+ relation = tail.
 
 The obejective loss function could be defined as :
 
 .. math::
   \mathcal{L} = \sum_{(h,l,t)\in S} (\sum_{(h',l,t')\in S'} [\gamma + d(h+l,t) - d'(h'+l,t')]_{+})
   
 Where S' is the negative samples generated (which are not real).
 
 7.6 Graph Embedding 
 -------------------
 
 The former descussed the node embedding, here we consider the embedding of the whole graph (for an example, for graph classification tasks) .
 Here shown some cases for realize it:
 
 * Simple summary :math:`z_{G} = \sum_{v\in G} z_{v}`.
 * Introduce a virtual node to represent the (sub)graph and run a standard graph embedding technique. (see *Li et al., Gated Graph Sequence Neural Networks (2016)*)
 * Anonymous walk embeddings : keep tracking the index of its first time visit in a random walk, other than the specific node.
 
 8. Graph Neural Networks
 =============================
 
 Here we learned :
 
 * GCN
 * Graph SAGE
 * GAT
 
 
 
