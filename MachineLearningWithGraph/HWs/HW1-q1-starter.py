################################################################################
# CS 224W (Fall 2019) - HW1
# Starter code for Question 1
# Last Updated: Sep 25, 2019
################################################################################

import snap
import numpy as np
import matplotlib.pyplot as plt

# Setup
erdosRenyi = None
smallWorld = None
collabNet = None


# Problem 1.1
def genErdosRenyi(N=5242, E=14484):
    """
    :param - N: number of nodes
    :param - E: number of edges

    return type: snap.PUNGraph
    return: Erdos-Renyi graph with N nodes and E edges
    """
    ############################################################################
    # TODO: Your code here!
    Graph = snap.PUNGraph.New()
    # Graph = snap.GenRndGnm(snap.PUNGraph, N, E)

    for i in range(N):
        Graph.AddNode(i)

    for i in range(E):
        id_1 = np.random.randint(N)
        id_2 = np.random.randint(N)
        Graph.AddEdge(id_1,id_2)

    ############################################################################
    return Graph


def genCircle(N=5242):
    """
    :param - N: number of nodes

    return type: snap.PUNGraph
    return: Circle graph with N nodes and N edges. Imagine the nodes form a
        circle and each node is connected to its two direct neighbors.
    """
    ############################################################################
    # TODO: Your code here!
    Graph = snap.PUNGraph.New()

    for i in range(N):
        Graph.AddNode(i)

    for i in range(N-1):
        Graph.AddEdge(i, i+1)
    Graph.AddEdge(N-1, 0)
    ############################################################################
    return Graph


def connectNbrOfNbr(Graph, N=5242):
    """
    :param - Graph: snap.PUNGraph object representing a circle graph on N nodes
    :param - N: number of nodes

    return type: snap.PUNGraph
    return: Graph object with additional N edges added by connecting each node
        to the neighbors of its neighbors
    """
    ############################################################################
    # TODO: Your code here!

    for i in range(N-2):
        Graph.AddEdge(i, i+2)
    Graph.AddEdge(N-2, 0)
    Graph.AddEdge(N-1, 1)

    # find all the nodes
    # for NI in Graph.Nodes():
    #     node_id = NI.GetId()
    #     # find the neighbors of the node
    #     for EI in NI.GetInEdges():
    #         # find the edges of this neighbor
    #         for NI_2 in Graph.Nodes():
    #             if NI_2.GetId() == EI :
    #                 for EI_2 in NI_2.GetInEdges():
    #                     if node_id == EI_2:
    #                         continue;
    #                     Graph.AddEdge(node_id, EI_2)

    ############################################################################
    return Graph


def connectRandomNodes(Graph, M=4000):
    """
    :param - Graph: snap.PUNGraph object representing an undirected graph
    :param - M: number of edges to be added

    return type: snap.PUNGraph
    return: Graph object with additional M edges added by connecting M randomly
        selected pairs of nodes not already connected.
    """
    ############################################################################
    # TODO: Your code here!

    N = Graph.GetNodes()
    i = 0
    while i < M:
        id_1 = np.random.randint(N)
        id_2 = np.random.randint(N)

        # check connection of the two nodes
        Nodes = snap.TIntV()
        Nodes.Add(id_1)
        Nodes.Add(id_2)
        results = snap.GetEdgesInOut(Graph, Nodes)
        if(results[0] == 0):
            Graph.AddEdge(id_1,id_2)
            i += 1

    ############################################################################
    return Graph


def genSmallWorld(N=5242, E=14484):
    """
    :param - N: number of nodes
    :param - E: number of edges

    return type: snap.PUNGraph
    return: Small-World graph with N nodes and E edges
    """
    Graph = genCircle(N)
    print("==> The circle graph has :", Graph.GetNodes(),"nodes and", Graph.GetEdges(), "edges")
    Graph = connectNbrOfNbr(Graph, N)
    print("==> The neigbor connected circle graph has :", Graph.GetNodes(),"nodes and", Graph.GetEdges(), "edges")
    Graph = connectRandomNodes(Graph, 4000)
    print("==> The small graph graph has :", Graph.GetNodes(),"nodes and", Graph.GetEdges(), "edges")
    return Graph


def loadCollabNet(path):
    """
    :param - path: path to edge list file

    return type: snap.PUNGraph
    return: Graph loaded from edge list at `path and self edges removed

    Do not forget to remove the self edges!
    """
    ############################################################################
    # TODO: Your code here!
    # Repeats are automatically ignored when loading an (un)directed graph
    Graph = snap.LoadEdgeList(snap.PUNGraph, path, 0, 1)
    # remove self-edges
    snap.DelSelfEdges(Graph)

    ############################################################################
    return Graph


def getDataPointsToPlot(Graph):
    """
    :param - Graph: snap.PUNGraph object representing an undirected graph

    return values:
    X: list of degrees
    Y: list of frequencies: Y[i] = fraction of nodes with degree X[i]
    """
    ############################################################################
    # TODO: Your code here!
    X, Y = [], []

    DegToCntV = snap.TIntPrV()
    snap.GetDegCnt(Graph, DegToCntV)
    for item in DegToCntV:
        #print("%d nodes with degree %d" % (item.GetVal2(), item.GetVal1()))
        X.append(item.GetVal1())
        Y.append(item.GetVal2())

    ############################################################################
    return X, Y


def Q1_1():
    """
    Code for HW1 Q1.1
    """
    global erdosRenyi, smallWorld, collabNet
    erdosRenyi = genErdosRenyi(5242, 14484)
    smallWorld = genSmallWorld(5242, 14484)
    collabNet = loadCollabNet("ca-GrQc.txt")
    print("==> The Real-World Collaboration Network has :", collabNet.GetNodes(),"nodes and", collabNet.GetEdges(), "edges")

    x_erdosRenyi, y_erdosRenyi = getDataPointsToPlot(erdosRenyi)
    plt.loglog(x_erdosRenyi, y_erdosRenyi, color = 'y', label = 'Erdos Renyi Network')

    x_smallWorld, y_smallWorld = getDataPointsToPlot(smallWorld)
    plt.loglog(x_smallWorld, y_smallWorld, linestyle = 'dashed', color = 'r', label = 'Small World Network')

    x_collabNet, y_collabNet = getDataPointsToPlot(collabNet)
    plt.loglog(x_collabNet, y_collabNet, linestyle = 'dotted', color = 'b', label = 'Collaboration Network')

    plt.xlabel('Node Degree (log)')
    plt.ylabel('Proportion of Nodes with a Given Degree (log)')
    plt.title('Degree Distribution of Erdos Renyi, Small World, and Collaboration Networks')
    plt.legend()
    plt.show()


# Execute code for Q1.1
Q1_1()


# Problem 1.2 - Clustering Coefficient

def calcClusteringCoefficientSingleNode(Node, Graph):
    """
    :param - Node: node from snap.PUNGraph object. Graph.Nodes() will give an
                   iterable of nodes in a graph
    :param - Graph: snap.PUNGraph object representing an undirected graph

    return type: float
    returns: local clustering coeffient of Node
    """
    ############################################################################
    # TODO: Your code here!
    C = 0.0
    C = snap.GetNodeClustCf(Graph, Node.GetId())
    ############################################################################
    return C

def calcClusteringCoefficient(Graph):
    """
    :param - Graph: snap.PUNGraph object representing an undirected graph

    return type: float
    returns: clustering coeffient of Graph
    """
    ############################################################################
    # TODO: Your code here! If you filled out calcClusteringCoefficientSingleNode,
    #       you'll probably want to call it in a loop here
    C = 0.0
    V = Graph.GetNodes()
    for NI in Graph.Nodes():
        Ci = calcClusteringCoefficientSingleNode(NI, Graph)
        C = C + Ci
    C = C / V
    ############################################################################
    return C

def Q1_2():
    """
    Code for Q1.2
    """
    C_erdosRenyi = calcClusteringCoefficient(erdosRenyi)
    C_smallWorld = calcClusteringCoefficient(smallWorld)
    C_collabNet = calcClusteringCoefficient(collabNet)

    print('Clustering Coefficient for Erdos Renyi Network: %f' % C_erdosRenyi)
    print('Clustering Coefficient for Small World Network: %f' % C_smallWorld)
    print('Clustering Coefficient for Collaboration Network: %f' % C_collabNet)


# Execute code for Q1.2
Q1_2()

# Clustering Coefficient for Erdos Renyi Network: 0.000898
# Clustering Coefficient for Small World Network: 0.284203
# Clustering Coefficient for Collaboration Network: 0.529636
