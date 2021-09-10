Convex in Slam
====================

Ordering
------------

To investiagte the sparity of the the system (graph). (ordering in ceres)

* use QR decomposition method. (connected with graph theory)
* Minimum Degree (MD) method, eliminate the node by the degree order.
* Approximate Minimum Degree (AMD). Not operate by node, but with group of nodes, which process faster.
* Reduced Camera System (mostly in visual SLAM, separate landmark points and camera pose, using Schur complement method). In some complicated cases, AMD achieve faster and better performance. (tested in some simple, ordinary cases, ordering doesn't perform faster than RCS).

Incremental BA
-----------------

* use new elements to update the olde hessian/jacobians (using QR based method), no need to update the full hessian at each iteration.
* filtering method (or slide window methods), marginalize the older states.
* Bayes tree (in iSAM), to investigate the graph structure, make it possible to update related nodes only.
