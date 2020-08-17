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

.. image:: images/transe.PNG
  :align: center
  :width: 90%

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

8.1 Introduction
---------------------

* Encoding function : network structure. (ML/DL)
* Similarity function : loss function.

Graph Neural Network (multiple layers of nonlinear transformations of graph structure) -> Graph convolution (:math:`\approx \sum_{i}w_{i}h_{i}`)

Graph Neural Networks (GNNs) are a class of neural network architectures used for deep learn-
ing on graph-structured data. Broadly, GNNs aim to generate high-quality embeddings of nodes
by iteratively aggregating feature information from local graph neighborhoods using neural net-
works; embeddings can then be used for recommendations, classication, link prediction or other
downstream tasks. Two important types of GNNs are GCNs (graph convolutional networks) and
GraphSAGE (graph sampling and aggregation).

8.2 GCN
------------------

The key-element of GCN is the neighborhood computation graph (neighborhood aggregation), shown as follows:

.. image:: images/aggregate_neighbors.png
   :align: center
   :width: 75%

And we could find the graph for all the nodes in the example graph:

.. image:: images/computation_graph.png
   :align: center
   :width: 90%

The basic structure is shown in the following image.

.. image:: images/computation_graph_for_a.png
   :align: center
   :width: 60%

* We don't need to much layers, as we don't want to capture the whole network, while we want to explore more the local properties.
* Each element block is a Graph convolution element, we could apply a summary/average/pooling/etc and following a neural network, finally apply a nonlinear activation function. For average GCN we have :

.. math::
  h_{v}^{k} = \sigma(w_{k}\sum_{u\in N(v)} \frac{h_{u}^{k-1}}{\mid N(v)\mid} + B_{k}h_{v}^{k-1} )

* Trainning of the network could using unsupervised method (last lecture), or supervised method using loss function, here we show an example of the node classification loss function (e.g. for application of drug-drug graph safe/toxic classification):

.. math::
  \mathcal{L} = \sum_{v\in V}y_{v}\log(\sigma(z_{v}^{T}\theta)) + (1-y_{v})\log(1-\sigma(z_{v}^{T}\theta))

* It has steps : **Message computation** (calculate H), **Aggregation** (:math:`D^{-1/2}AD^{-1/2}`), **Update**(in GCNs, a multi-layer perceptron (MLP) is used), **Pooling** (usually done for the purposes of graph classication).

Matirx representation:

.. math::
  H^{k} = D^{-1}AH^{k-1} = D^{-1/2}AD^{-1/2}H^{k-1}

For GCN the following equation is used:

.. math::
  h^{k} = \sigma(D^{-1/2}AD^{-1/2}h^{k-1}W^{k})

where :math:`\sigma` is non-linear function, it could be activation function, drop-out function , etc. and :math:`W^{k}` is the
learnable parameter.

8.3 Graph SAGE
---------------------

It introduce a more general aggregation function choices here.

.. math::
  h_{v}^{k} = \sigma([W_{k}AGG( \{ h_{u}^{k-1}, \forall u\in N(v)  \})  ,B_{k}h_{v}^{k-1} ])

There are some commonly used aggregation functions:

* Mean: :math:`AGG = \sum_{u\in N(v)}h_{u}^{k-1}/\mid N(v)\mid`.
* Pooling : :math:`AGG = \gamma (\{ Qh_{u}^{k-1}, \forall u\in N(v)  \})` .
* LSTM : (applied to several randomly reshuffled neighbors) :math:`AGG=LSTM(\{ h_{u}^{k-1}, \forall u\in \pi(N(v))\})`

.. image:: images/gcn_sudocode.PNG
   :align: center
   :width: 90%

8.4 Graph Attention Networks
---------------------------

In the GCN, we take the summary of all the neighbors with the same weight :math:`1/\mid N(v)\mid`, we also equally count the neighbors in Graph SAGE too.
So the motivation here is to dynamically choose different weights for nodes, based on **Attention Mechanism**.

.. math::
  e_{vu} = a(W_{k}h_{u}^{k-1}, W_{k}h_{v}^{k-1})

Then apply a softmax for the normalization of the weights :

.. math::
  \alpha_{vu} = exp(e_{vu})/(\sum_{k\in N(v)} exp(e_{vk}))

.. math::
  \alpha_{vu} = \exp(LeakyReLU(e_{vu}))/(\sum_{k\in N(v)} \exp(LeakyReLU(e_{vk})))

Therefore we have :

.. math::
  h_{v}^{k} = \sigma(\sum_{u\in N(v)} \alpha_{vu}W_{k}h_{u}^{k-1})


Example : PinSAGE.
