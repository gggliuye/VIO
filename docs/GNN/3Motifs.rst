3. Motifs and Structral Rules in Network
=======================================

3.1 Subgraphs
--------------------------

The idea here is to characterize and discriminiate networks using the **subgraphs**.

* **Motifs** : Global level subgraph metric. Recurring significant patterns of inter-connection. Using the parameter **Significance profile**, count the appearance of a motif compared with a randomly generated model [1]_ with similar properties.

.. math::
  Z_{i} = (N_{i}^{real} - \bar{N}_{i}^{rand})/std(N_{i}^{rand})

.. math::
  SP_{i} = Z_{i}/(\sum_{j}Z_{j}^{2})

* **Graphlets** : Node level subgraph metric. Node feature vectors. Using the parameter **Graphlet degree vector** (GDV) [2]_ , reflects the node's level network topology.

.. [1] Random model generated using *Configuration model* or *Switching*, will have same #edges, #nodes, and same distribution of degree (to try to capture the properties of the real network).

.. [2] A vector for each node, the length is # possible orbit positions of different graphlet(usually take graphlets of 5-8 nodes), each element is the number of orbit positions of different graphlet types that the node participates.

*Find motifs and graphlets* in a graph. consists of two steps:

* **Enumerating** : introduces method *Exact subgraph enumeration* (ESU) which find two set :math:`V_{subgraph}` for the current subgraph nodes and :math:`V_{extansion}` for the candidate new nodes. It will iteratively adding new nodes from :math:`V_{extansion}` to :math:`V_{subgraph}` until reaches the objective properties (#nodes).

* **Counting** : to filter duplicated graphs, using isomorphisms test (Mckay's nauty algorithm 1989), try to find a mapping to make two graphs identical.


3.2 Role
---------------

**Role** : 'functions' of nodes in a network. (~ profession, job). Measured by structural properties.

**Communications/Groups** : well-connected nodes.

Find the structural equivalent nodes. Or find structural 'similar' nodes, which could be used to identify roles.

**RoIX** (Role eXtraction : Structural Role Extraction & Mining in Large Graphs) :
an unsupervised learning approach for automatically extracting structural roles from general network data sets.
find feature vectors for all the nodes to help identify roles. Obtained by the Adjacency matrix. The vector captures
features of different scale : **local** (properties of the node itself), **egonet** (properties of neighbors, similar to a
convolution step), and **recursive** (mean/sum of neighbors, similar to a pooling step). Followed by a clustering process,
we could find nodes grouped by similar roles.
