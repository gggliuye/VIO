11. PageRank
=======================
PageRank is a method for ranking webpages by importance using link structure in the web graph.
This is a commonly used `algorithm <http://ilpubs.stanford.edu:8090/422/>`_ for web search popularized by Google.

11.1 Web
---------------

Web is directed graph with :

* Nodes as web pages. Consider only static webpages, Ignore dark matter on the web (i.e. inaccessible material, pages behind firewalls).
* Edges as the hyperlinks. Assume all links are navigable. Transactional links (eg: like, buy, follow etc.) are not considered.

**Theorem** : Any directed graph can be represented as a combination of these two types : Strongly connected graphs, and
Directed Acyclic Graph (DAG: A graph where there are no cycles and if u can reach v, then v cannot reach u).

**Find SCC** (where G' is the graph G with inversed edges):

.. math::
  SCC = Out(v) \cap In(v) = Out(v,G) \cap Out(v, G')

**Bowtie structure** of network :

.. image:: images/BowTie_2.png
   :align: center
   :width: 60%

11.2 PageRank
-----------------------

Ranking nodes on the graph. Think **links as votes** (with in-links' edges having higher weight). and as a recursive process.

.. math::
  r_{j} = \sum_{i\to j}\frac{r_{i}}{d_{i,out}}, \quad r_{j\to k} = \frac{r_{j}}{d_{j,out}}

We could matrix to represent the process, with M an adjacency matrix that is column stochastic :

.. math::
  r = Mr

**Interpretation** : It’s modeling the stationary distribution of the random walker process on the graph.
Or we could see it as the equilibrium of a energy flow.

**Power law iteration** : Limiting distribution = principal eigenvector of M = PageRank.
Run the process :math:`r^{t+1} = Mr^{t}` until convergence.


**Problems**:

* **Dead ends** leak out -> Add a random teleports (add 1/N to each element of the adjacency matrix).
* **Spider traps** self-loop -> random jump with probability :math:`1-\beta`.

By solving the problems using the upper methods, we will have :

.. math::
  r_{j} = \sum_{i\to j}\beta \frac{r_{i}}{d_{i}} + (1-\beta)\frac{1}{N}

.. math::
  r = \beta Mr + \frac{[1-beta]}{N} = Ar

11.3 Examples
----------------------

**Bipartite : User-to-Item graph**

.. image:: images/QPPR.png
   :align: center

**Summary**:

* Normal pagerank: Teleportation vector is uniform.
* Personalized PageRank: Teleport to a topic specific set of pages. Nodes can have different probabilities of surfer landing there.
* Random walk with restarts: Topic specific pagerank where teleport is always to the same node. In this case, we don’t need power iteration we can just use random walk and its very fast and easy.

11.4 HW3 Q1
-----------------

**Question 1.1** : We could roughly tell if a node is belong to SCC, IN or OUT, by the difference pf the number of nodes in the
inward BFS tree and the outward BFS tree. If the difference is relatively small, it should be in the SCC. If large, it will be in IN or OUT.
The result of this question is ::

  GT: Node 2018 in the SCC :  False
  A lot more nodes in the outward BFS tree, Node 2018 should be in IN set.
  GT: Node 224 in the SCC :  True
  Inward and outward trees have roughly the same size, Node 224 should be in SCC set.

**Question 1.2** : The question ask us to plot four images, while I plot two image by combining the inward/outward in the same image, to better illustrate the result.
And I run 1000 random walk to get a better simulation result (even it took a lot more times).

.. image:: images/bowtie_q1.png
   :align: center

* I found the two plots (in/out) of the Epinions social network is similar to each other. Which means that most of the nodes in the graph are in the SCC, as social network is closely connected.
* While in the email graph, the difference is huge, larger part of the nodes has many outlinks, while much fewer nodes has many inlinks. which means the graph has a relative small SCC and OUT, while a very large IN group. 
