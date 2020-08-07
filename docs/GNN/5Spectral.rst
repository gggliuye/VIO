5. Spectral Clustering
=================================

For community detection using eigenvectors and eigenvalues.

* Pre-processing : matrix representation : **Laplacian matrix**.
* Decomposition : **second** eigenvalue and eigenvector.
* Grouping : take the **sign** / maximize **eigengap**.


5.1 Concepts
-----------------

**Cut** : :math:`cut(A,B) =\sum_{i\in A, j\in B} w_{i,j}`, but it sometimes could not
capture the inner properties of the groups.

**Conductance** : :math:`\phi(A,B) = cut(A,B)/ \min(vol(A), vol(B))`, will produce more
balanced partitions. While optimize it is NP-hard.

**Spectral graph partitioning** : using spectral properties to have approximation to
conductance optimization problem.


5.2 Spectral Clustering
-------------------------

* The property of the adjacency matrix A, with the label vector x.
* Introudce the **Laplacian matrix** :math:`L = D-A`.
    * L has degrees in the disagonal part, and negative edge weight at the other part.
    * Non-negative real eigenvalues, and real eigenvectors.
    * We have :math:`L \mathbb{1} = 0`, :math:`\lambda_{1} = 0`, and :math:`\mathbb{1}` is its corresponding eigenvector.
* Use normalized laplacian matrix :math:`\bar{L} = D^{-1/2}LD^{-1/2}` for better performance. (see the proof in HW1-Q4)

The problem could be reformed to :

.. math::
  \begin{align*}
  &minimize \quad x^{T}Lx \\
  &subject\ to \quad x^{T}x = 1, \ x^{T}\mathbb{1} = 0
  \end{align*}

We could prove that its optimal value is :math:`\lambda_{2}`, and we also
have :math:`x^{T}Lx = \sum_{(i,j)\in E}(x_{i}-x_{j})^{2}`. (See the details in
my hand writing notes or in the course slides.)
Then the problem reduce to find the second eigenvalue and eigenvector of the Laplacian matrix.

**Relationship with modularity** see in HW1-Q4.

**Questions** : The upper problem is actually a L2-minimization problem.
While the problem is better modeled as a cardinality problem, should we use l1 heuristic to get a better result?

5.3 K Clustering
--------------------

* **Find K**: maximize the **eigengap** :math:`\Delta k = \mid \lambda_{k} - \lambda_{k-1}\mid`
* Select the K eigenvectors, then apply K-mean clustering.

5.4 Motif-based spectral clustering
-----------------------------

Generalize the 'edge' definitation to motif level.

* Motif cut.
* Motif count.

.. math::
  \phi_{M}(S) = \frac{\#motifs\ cut}{motif\ volume}

Example in the gene regulation show the **feedward loops**, exactly the same as shown in the course
Human Behavioral Biology BIO150.

5.5 HW1 - q4
------------------

`Hand writing homework <https://github.com/gggliuye/VIO/blob/master/MachineLearningWithGraph/HWs/HW1-q4.pdf>`_ :
**Normalized laplacian matrix** and **Relationship with modularity**.
