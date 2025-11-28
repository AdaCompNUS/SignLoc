

import networkx as nx
import pickle
import numpy as np 


def addActionEdge(G, u, v, numActions=4):

    dirc = G.nodes[v]["pos"] - G.nodes[u]["pos"]
    dist = np.linalg.norm(dirc)
    action = 0

    G.add_edge(u, v, type="action", weight=dist)
    G.add_edge(v, u, type="action", weight=dist)

    return G

def findConnectedEdges(G, node, types):

    edges = [(u, v) for u, v, d in G.edges(node, data=True) if d["type"] in types]

    return edges




def load_graph(navGraphPath):


    G = pickle.load(open(navGraphPath, 'r+b'))
    graphdict = {}

    for n, d in G.nodes(data=True):

        if d["type"] in ["intersection", "terminal"]:
            edges = findConnectedEdges(G, n, ["hierarchical"])
            edges = [(u, v) for u, v in edges if G.nodes[v]["type"] == "room"]
            for (u, v) in edges:
                G[u][v]["weight"] = 0

        elif d["type"] == "room":
            edges = findConnectedEdges(G, n, ["hierarchical"])
            edges = [(u, v) for u, v in edges if G.nodes[v]["type"] == "floor"]
            for (u, v) in edges:
                G[u][v]["weight"] = 0

        elif d["type"] == "floor":
            edges = findConnectedEdges(G, n, ["hierarchical"])
            edges = [(u, v) for u, v in edges if G.nodes[v]["type"] == "building"]
            for (u, v) in edges:
                G[u][v]["weight"] = 0


    for n, d in G.nodes(data=True):
        if "category" in d and d["category"] in ["escalator", "stairs", "lift"]:
            edges = findConnectedEdges(G, n, ["virtual"])
            for (u, v) in edges:
                G.remove_edge(u, v)
                G.remove_edge(v, u)
                G = addActionEdge(G, u, v)

        if "name" in d:
            roomname = ' '.join(d["name"])
            if len(roomname) < 2:
                G.nodes[n]["name"] = [""]
            else:
                G.nodes[n]["name"] = roomname

 

    nodesWithoutData = [n for n, d in G.nodes(data=True)]
    nodesData = [d for n, d in G.nodes(data=True)]

    edgesWithoutData = [(u, v) for u, v, d in G.edges(data=True)]
    edgesData = [d for u, v, d in G.edges(data=True)]

    paths = dict(nx.all_pairs_dijkstra_path(G, weight='weight'))
    distances = dict(nx.all_pairs_dijkstra_path_length(G, weight='weight'))

    graphdict["nodes"] = nodesWithoutData
    graphdict["nodesData"] = nodesData
    graphdict["edges"] = edgesWithoutData
    graphdict["edgesData"] = edgesData

    graphdict["paths"] = paths
    graphdict["distances"] = distances

   

    return graphdict











