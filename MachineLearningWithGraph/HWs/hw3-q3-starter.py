###############################################################################
# CS 224W (Fall 2019) - HW3
# Starter code for Problem 3
###############################################################################

import snap
import numpy as np
import matplotlib.pyplot as plt

# Setup
num_voters = 10000
decision_period = 10


def read_graphs(path1, path2):
    """
    :param - path1: path to edge list file for graph 1
    :param - path2: path to edge list file for graph 2

    return type: snap.PUNGraph, snap.PUNGraph
    return: Graph 1, Graph 2
    """
    ###########################################################################
    # TODO: Your code here!

    Graph1 = snap.LoadEdgeList(snap.PUNGraph, path1, 0, 1)
    Graph2 = snap.LoadEdgeList(snap.PUNGraph, path2, 0, 1)
    #print(Graph1.GetNodes(), Graph2.GetNodes())

    ###########################################################################
    return Graph1, Graph2


def initial_voting_state(Graph):
    """
    Function to initialize the voting preferences.

    :param - Graph: snap.PUNGraph object representing an undirected graph

    return type: Python dictionary
    return: Dictionary mapping node IDs to initial voter preference
            ('A', 'B', or 'U')

    Note: 'U' denotes undecided voting preference.

    Example: Some random key-value pairs of the dict are
             {0 : 'A', 24 : 'B', 118 : 'U'}.
    """
    voter_prefs = {}
    ###########################################################################
    # TODO: Your code here!

    for NI in Graph.Nodes():
        node_id = NI.GetId()
        if(node_id%10 < 4):
            voter_prefs[node_id] = 'A'
        elif(node_id%10 < 8):
            voter_prefs[node_id] = 'B'
        else:
            voter_prefs[node_id] = 'U'

    ###########################################################################
    assert(len(voter_prefs) == num_voters)
    return voter_prefs


def iterate_voting(Graph, init_conf):
    """
    Function to perform the 10-day decision process.

    :param - Graph: snap.PUNGraph object representing an undirected graph
    :param - init_conf: Dictionary object containing the initial voting
                        preferences (before any iteration of the decision
                        process)

    return type: Python dictionary
    return: Dictionary containing the voting preferences (mapping node IDs to
            'A','B' or 'U') after the decision process.

    Hint: Use global variables num_voters and decision_period to iterate.
    """
    curr_conf = init_conf.copy()
    curr_alternating_vote = 'A'
    ###########################################################################
    # TODO: Your code here!
    for iter in range(decision_period):
        for i in range(num_voters):
            if(init_conf[i] != 'U'):
                continue;

            # the undecided voter make their decisions
            NI = Graph.GetNI(i)
            count_a = 0
            count_b = 0
            for Id in NI.GetOutEdges():
                if(curr_conf[Id] == 'A'):
                    count_a+=1
                elif(curr_conf[Id] == 'B'):
                    count_b+=1
            if(count_a > count_b):
                curr_conf[i] = 'A'
            elif(count_a < count_b):
                curr_conf[i] = 'B'
            else:
                curr_conf[i] = curr_alternating_vote
                if(curr_alternating_vote == 'A'):
                    curr_alternating_vote = 'B'
                else:
                    curr_alternating_vote = 'A'

    ###########################################################################
    return curr_conf


def sim_election(Graph):
    """
    Function to simulate the election process, takes the Graph as input and
    gives the final voting preferences (dictionary) as output.
    """
    init_conf = initial_voting_state(Graph)
    conf = iterate_voting(Graph, init_conf)
    return conf


def winner(conf):
    """
    Function to get the winner of election process.
    :param - conf: Dictionary object mapping node ids to the voting preferences

    return type: char, int
    return: Return candidate ('A','B') followed by the number of votes by which
            the candidate wins.
            If there is a tie, return 'U', 0
    """
    ###########################################################################
    # TODO: Your code here!
    count_a = 0
    count_b = 0
    for i in range(num_voters):
        if(conf[i] == 'A'):
            count_a += 1
        elif(conf[i] == 'B'):
            count_b += 1

    if(count_a == count_b):
        return ['U', 0]
    elif(count_a > count_b):
        return ['A', count_a]
    else:
        return ['B', count_b]
    ###########################################################################


def Q1():
    print ("\nQ1:")
    Gs = read_graphs('graph1.txt', 'graph2.txt')    # List of graphs

    # Simulate election process for both graphs to get final voting preference
    final_confs = [sim_election(G) for G in Gs]

    # Get the winner of the election, and the difference in votes for both
    # graphs
    res = [winner(conf) for conf in final_confs]

    for i in range(2):
        print("In graph",i+1,", candidate",res[i][0],"wins by",res[i][1],"votes")


def Q2sim(Graph, k):
    """
    Function to simulate the effect of advertising.
    :param - Graph: snap.PUNGraph object representing an undirected graph
             k: amount to be spent on advertising

    return type: int
    return: The number of votes by which A wins (or loses), i.e. (number of
            votes of A - number of votes of B)

    Hint: Feel free to use initial_voting_state and iterate_voting functions.
    """
    ###########################################################################
    # TODO: Your code here!
    init_conf = initial_voting_state(Graph)
    num_reached_voters = int(k/100)
    for i in range(3000, 3000+num_reached_voters):
        init_conf[i] = 'A'

    conf = iterate_voting(Graph, init_conf)

    count_a = 0
    count_b = 0
    for i in range(num_voters):
        if(conf[i] == 'A'):
            count_a += 1
        elif(conf[i] == 'B'):
            count_b += 1
    return count_a - count_b
    ###########################################################################


def find_min_k(diffs):
    """
    Function to return the minimum amount needed for A to win
    :param - diff: list of (k, diff), where diff is the value by which A wins
                   (or loses) i.e. (A-B), for that k.

    return type: int
    return: The minimum amount needed for A to win
    """
    ###########################################################################
    # TODO: Your code here!
    diffs_array = np.asarray(diffs)
    ks = diffs_array[:,0][diffs_array[:,1] > 0]
    return min(ks)
    ###########################################################################


def makePlot(res, title):
    """
    Function to plot the amount spent and the number of votes the candidate
    wins by
    :param - res: The list of 2 sublists for 2 graphs. Each sublist is a list
                  of (k, diff) pair, where k is the amount spent, and diff is
                  the difference in votes (A-B).
             title: The title of the plot
    """
    Ks = [[k for k, diff in sub] for sub in res]
    res = [[diff for k, diff in sub] for sub in res]
    ###########################################################################
    # TODO: Your code here!

    ###########################################################################
    plt.plot(Ks[0], res[0], ':', label='graph1')
    plt.plot(Ks[1], res[1], ':', label='graph2')
    plt.xlabel('Amount spent ($)')
    plt.ylabel('#votes for A - #votes for B')
    plt.title(title)
    plt.legend()
    plt.show()


def Q2():
    print ("\nQ2:")
    # List of graphs
    Gs = read_graphs('graph1.txt', 'graph2.txt')

    # List of amount of $ spent
    Ks = [x * 1000 for x in range(1, 10)]

    # List of (List of diff in votes (A-B)) for both graphs
    res = [[(k, Q2sim(G, k)) for k in Ks] for G in Gs]

    # List of minimum amount needed for both graphs
    min_k = [find_min_k(diff) for diff in res]

    formatString = "On graph {}, the minimum amount you can spend to win is {}"
    for i in range(2):
        print(formatString.format(i + 1, min_k[i]))

    makePlot(res, 'TV Advertising')


def Q3sim(Graph, k):
    """
    Function to simulate the effect of a dining event.
    :param - Graph: snap.PUNGraph object representing an undirected graph
             k: amount to be spent on the dining event

    return type: int
    return: The number of votes by which A wins (or loses), i.e. (number of
            votes of A - number of votes of B)

    Hint: Feel free to use initial_voting_state and iterate_voting functions.
    """
    ###########################################################################
    # TODO: Your code here!

    init_conf = initial_voting_state(Graph)
    num_roller_voters = int(k/100)

    degrees = np.zeros(num_voters)
    for i in range(num_voters):
        NI = Graph.GetNI(i)
        degree = NI.GetInDeg()
        degrees[i] = degree

    sort_idx = np.argsort(degrees)
    for i in range(num_roller_voters):
        idx = num_voters - i - 1
        init_conf[sort_idx[idx]] = 'A'

    conf = iterate_voting(Graph, init_conf)

    count_a = 0
    count_b = 0
    for i in range(num_voters):
        if(conf[i] == 'A'):
            count_a += 1
        elif(conf[i] == 'B'):
            count_b += 1
    return count_a - count_b
    ###########################################################################


def Q3():
    print ("\nQ3:")
    # List of graphs
    Gs = read_graphs('graph1.txt', 'graph2.txt')

    # List of amount of $ spent
    Ks = [x * 1000 for x in range(1, 10)]

    # List of (List of diff in votes (A-B)) for both graphs
    res = [[(k, Q3sim(G, k)) for k in Ks] for G in Gs]

    # List of minimum amount needed for both graphs
    min_k = [find_min_k(diff) for diff in res]

    formatString = "On graph {}, the minimum amount you can spend to win is {}"
    for i in range(2):
        print(formatString.format(i + 1, min_k[i]))

    makePlot(res, 'Wining and Dining')


def Q4():
    """
    Function to plot the distributions of two given graphs on a log-log scale.
    """
    print ("\nQ4:")
    ###########################################################################
    # TODO: Your code here!
    ###########################################################################


def main():
    Q1()
    Q2()
    Q3()
    Q4()


if __name__ == "__main__":
    main()
