import snap
import random
import numpy as np
import matplotlib.pyplot as plt

def load_graph(name):
    '''
    Helper function to load graphs.
    Use "epinions" for Epinions graph and "email" for Email graph.
    Check that the respective .txt files are in the same folder as this script;
    if not, change the paths below as required.
    '''
    if name == "epinions":
        G = snap.LoadEdgeList(snap.PNGraph, "soc-Epinions1.txt", 0, 1)
    elif name == 'email':
        G = snap.LoadEdgeList(snap.PNGraph, "email-EuAll.txt", 0, 1)
    else:
        raise ValueError("Invalid graph: please use 'email' or 'epinions'.")
    return G

def q1_1():
    '''
    You will have to run the inward and outward BFS trees for the
    respective nodes and reason about whether they are in SCC, IN or OUT.
    You may find the SNAP function GetBfsTree() to be useful here.
    '''

    ##########################################################################
    #TODO: Run outward and inward BFS trees from node 2018, compare sizes
    #and comment on where node 2018 lies.
    G = load_graph("email")
    #Your code here:
    print("==> Start question 1.1 :")
    NodeId = 2018
    MxScc = snap.GetMxScc(G)
    nodes_scc = []
    for NI in MxScc.Nodes():
        nodes_scc.append(NI.GetId())
        #print("node: (%d)" % (NI.GetId()))
    print(" GT: Node",NodeId,"in the SCC : ", NodeId in nodes_scc)
    BfsTreeOut = snap.GetBfsTree(G, NodeId, True, False)
    BfsTreeIn = snap.GetBfsTree(G, NodeId, False, True)
    difference_nodes = BfsTreeOut.GetNodes() - BfsTreeIn.GetNodes()
    threshold = G.GetNodes() * 0.15
    #print(difference_nodes, threshold)
    if(difference_nodes > threshold):
        print("A lot more nodes in the outward BFS tree, Node", NodeId, "should be in IN set.")
    elif (difference_nodes < -threshold):
        print("A lot more nodes in the inward BFS tree, Node", NodeId, "should be in Out set.")
    else  :
        print("Inward and outward trees have roughly the same size, Node", NodeId, "should be in SCC set.")

    ##########################################################################

    ##########################################################################
    #TODO: Run outward and inward BFS trees from node 224, compare sizes
    #and comment on where node 224 lies.
    G = load_graph("epinions")
    #Your code here:
    NodeId = 224
    MxScc = snap.GetMxScc(G)
    nodes_scc = []
    for NI in MxScc.Nodes():
        nodes_scc.append(NI.GetId())
        #print("node: (%d)" % (NI.GetId()))
    print(" GT: Node",NodeId,"in the SCC : ", NodeId in nodes_scc)
    BfsTreeOut = snap.GetBfsTree(G, NodeId, True, False)
    BfsTreeIn = snap.GetBfsTree(G, NodeId, False, True)
    difference_nodes = BfsTreeOut.GetNodes() - BfsTreeIn.GetNodes()
    threshold = G.GetNodes() * 0.15
    #print(difference_nodes, threshold)
    if(difference_nodes > threshold):
        print("A lot more nodes in the outward BFS tree, Node", NodeId, "should be in IN set.")
    elif (difference_nodes < -threshold):
        print("A lot more nodes in the inward BFS tree, Node", NodeId, "should be in Out set.")
    else  :
        print("Inward and outward trees have roughly the same size, Node", NodeId, "should be in SCC set.")





    ##########################################################################

    print ('1.1: Done!\n')


def PlotRandomBFS(Graph, num_test = 100):
    Rnd = snap.TRnd(42)
    Rnd.Randomize()
    num_out = []
    num_in = []
    for i in range(0,num_test):
        NodeId = Graph.GetRndNId(Rnd)
        BfsTreeOut = snap.GetBfsTree(Graph, NodeId, True, False)
        BfsTreeIn = snap.GetBfsTree(Graph, NodeId, False, True)
        num_out.append(np.log(BfsTreeOut.GetNodes()))
        num_in.append(np.log(BfsTreeIn.GetNodes()))
    num_out.sort()
    num_in.sort()
    return num_out, num_in


def q1_2():
    '''
    For each graph, get 100 random nodes and find the number of nodes in their
    inward and outward BFS trees starting from each node. Plot the cumulative
    number of nodes reached in the BFS runs, similar to the graph shown in
    Broder et al. (see Figure in handout). You will need to have 4 figures,
    one each for the inward and outward BFS for each of email and epinions.

    Note: You may find the SNAP function GetRndNId() useful to get random
    node IDs (for initializing BFS).
    '''
    ##########################################################################
    #TODO: See above.
    #Your code here:

    print("==> Start question 1.2 :")
    num_test = 100
    x = np.arange(0,1,1/num_test)

    G = load_graph("email")
    num_out, num_in = PlotRandomBFS(G, num_test)

    plt.subplot(1,2,1)
    plt.plot(x,num_out, label="outlinks")
    plt.plot(x,num_in, label="inlinks")
    plt.title("email : Reachability using inlinks/outlinks")
    plt.xlabel("frac. of starting nodes")
    plt.ylabel("log. number of nodes reached")
    plt.legend()

    G = load_graph("epinions")
    num_out, num_in = PlotRandomBFS(G, num_test)

    plt.subplot(1,2,2)
    plt.plot(x,num_out, label="outlinks")
    plt.plot(x,num_in, label="inlinks")
    plt.title("epinions : Reachability using inlinks/outlinks")
    plt.xlabel("frac. of starting nodes")
    plt.ylabel("log. number of nodes reached")
    plt.legend()

    plt.show()


    ##########################################################################
    print('1.2: Done!\n')

def q1_3_grpah(Graph):
    n_nodes = Graph.GetNodes()
    MxWcc = snap.GetMxWcc(Graph)
    MxScc = snap.GetMxScc(Graph)
    n_MxWcc = MxWcc.GetNodes()
    n_MxScc = MxScc.GetNodes()
    print(" TOTAL          : ", n_nodes)
    print(" DISCONNECTED   : ", n_nodes-n_MxWcc)
    print(" SCC            : ", n_MxScc)

    SCC_nodes = []
    for NI in MxScc.Nodes():
        SCC_nodes.append(NI.GetId())

    num_test = 100
    random_sampled_scc = random.sample(SCC_nodes, num_test)

    num_out = []
    num_in = []
    for i in range(0, num_test):
        NodeId = random_sampled_scc[i]
        BfsTreeOut = snap.GetBfsTree(Graph, NodeId, True, False)
        BfsTreeIn = snap.GetBfsTree(Graph, NodeId, False, True)
        num_out.append(BfsTreeOut.GetNodes()) # roughly SCC + OUT
        num_in.append(BfsTreeIn.GetNodes()) # roughly SCC + IN
    num_out.sort()
    num_in.sort()

    print(" OUT            : ", num_out[-1]-n_MxScc)
    print(" IN             : ", num_in[-1]-n_MxScc)

    num_tendrils = n_MxWcc - n_MxScc - (num_out[-1]-n_MxScc) - (num_in[-1]-n_MxScc)
    print(" TENDRILS+TUBES : ", num_tendrils)

def q1_3():
    '''
    For each graph, determine the size of the following regions:
        DISCONNECTED
        IN
        OUT
        SCC
        TENDRILS + TUBES

    You can use SNAP functions GetMxWcc() and GetMxScc() to get the sizes of
    the largest WCC and SCC on each graph.
    '''
    ##########################################################################
    #TODO: See above.
    #Your code here:

    print("==> Start question 1.3 :")
    G = load_graph("email")
    print(" --- graph Email ---")
    q1_3_grpah(G)

    G = load_graph("epinions")
    print(" --- graph Epinions ---")
    q1_3_grpah(G)



    ##########################################################################
    print('1.3: Done!\n')


def TestFracPath(Graph, num_test = 1000):
    Rnd = snap.TRnd(42)
    Rnd.Randomize()
    count = 0
    i = 0
    while i < num_test:
        NodeId1 = Graph.GetRndNId(Rnd)
        NodeId2 = Graph.GetRndNId(Rnd)
        if(NodeId1 != NodeId2):
            Length = snap.GetShortPath(Graph, NodeId1, NodeId2, True)
            if(Length > 0):
                count += 1
            i += 1
    print(" fraction of reachable pairs", count/num_test)

def q1_4():
    '''
    For each graph, calculate the probability that a path exists between
    two nodes chosen uniformly from the overall graph.
    You can do this by choosing a large number of pairs of random nodes
    and calculating the fraction of these pairs which are connected.
    The following SNAP functions may be of help: GetRndNId(), GetShortPath()
    '''
    ##########################################################################
    #TODO: See above.
    #Your code here:

    print("==> Start question 1.4 :")
    G = load_graph("email")
    print(" --- graph Email ---")
    TestFracPath(G)

    G = load_graph("epinions")
    print(" --- graph Epinions ---")
    TestFracPath(G)

    ##########################################################################
    print('1.4: Done!\n')

if __name__ == "__main__":
    q1_1()
    q1_2()
    q1_3()
    q1_4()
    print("Done with Question 1!\n")
