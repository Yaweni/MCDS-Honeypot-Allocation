#include "GameGraph.h"
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <random>
#include <queue>
#include <iostream>
#include <iterator>
using namespace std;

Edge::Edge(unsigned int id, Node* src, Node* dst, double normalCost, double honeypotCost) : id(id), src(src), dst(dst),
normalCost(normalCost),
honeypotCost(
    honeypotCost) {}

Node::Node(unsigned int id) : id(id) {}

GameGraph::GameGraph(unsigned int numNodes) : numNodes(numNodes), numEdges(0) {
    for (unsigned int i = 0; i < numNodes; i++) {
        nodes.emplace_back(i);
    }
    target = &nodes[numNodes - 1];
}

GameGraph GameGraph::findMCDS()
{
    // Create a set to store the MCDS
    set<Node*> mcds;
    set<int> ids;


    vector<Node>  oldNodes = nodes;
    // Assume you have a vector<Node> called nodeVec
    vector<Node*> nodePtrVec; // Create a new vector of pointers
    for (Node& n : oldNodes) { // Loop over the original vector of nodes
        nodePtrVec.push_back(&n); // Add the address of each node to the new vector
    }
    //cout<<"Number of nodes "<<nodePtrVec.size()<<endl;

    // Create a vector to store the nodes sorted by their degrees
    vector<Node*> nodes = nodePtrVec; // Copy the nodes from the graph
    sort(nodes.begin(), nodes.end(), [](const Node* a, const Node* b) { return a->out.size() > b->out.size(); }); // Sort them by their degrees

    // Create a set to store the covered nodes
    set<int> covered;
    vector<Node*> nodes2 = nodes;
    // Loop until all nodes are covered
    while (covered.size() != numNodes) {
        // Pop the node with the highest degree from the vector

        Node* u = nodes2.front();
        nodes2.erase(nodes2.begin());

        set<int> u_neighbors, result, intersect;
        // Mark it and its neighbors as covered
        for (int i(0); i<int(edges2.size()); i++) {
            if (edges2[i].src->id == u->id) {
                u_neighbors.insert(edges2[i].dst->id);
                covered.insert(edges2[i].dst->id);
            }

        }
        covered.insert(u->id);
        set_difference(covered.begin(), covered.end(), u_neighbors.begin(), u_neighbors.end(), inserter(result, result.begin()));
        set_intersection(ids.begin(), ids.end(), u_neighbors.begin(), u_neighbors.end(), inserter(intersect, intersect.begin()));

        if (result.size() > 0) {
            ids.insert(u->id);
        }
    }
    ids.insert(numNodes - 1);
    cout << "[ ";
    for (Node* n : nodes) {
        if (ids.find(n->id) != ids.end()) {
            cout << n->id << ",";
            mcds.insert(n);
        }
    }
    cout << "] ";
    // Create a new game graph with the nodes in the MCDS
    GameGraph mcdsGraph = GameGraph(mcds.size());
    // Loop over the nodes in the MCDS
    set<int> drawn;
    for (Node* u : mcds) {
        vector<Edge> edges_u, edges_nu;
        for (int i(0); i<int(edges2.size()); i++) {

            if (((edges2[i].src->id) == (u->id)) & ids.find(edges2[i].dst->id) != ids.end()) {
                edges_u.emplace_back(edges2[i]);
            }
        }

        //edges_u.resize(distance(edges_u.begin(),uniques));
        bool connected = false;
        for (Edge j : edges_u) {
            if (ids.find(j.dst->id) != ids.end() & ids.find(j.src->id) != ids.end()) {
                set<int>::iterator iter1 = ids.find(j.dst->id);
                set<int>::iterator iter2 = ids.find(j.src->id);
                int index1 = distance(ids.begin(), iter1);
                int index2 = distance(ids.begin(), iter2);
                connected = true;
                // Connect v and w with the same costs as e
                mcdsGraph.connect(mcdsGraph[index2], mcdsGraph[index1], j.normalCost, j.honeypotCost);
                //mcdsGraph.connect(mcdsGraph[index1], mcdsGraph[index2], j.normalCost, j.honeypotCost);
            }
            
            }
        if (connected == false) {
            set<int>::iterator iter2 = ids.find(u->id);
            int index2 = distance(ids.begin(), iter2);
            double cost = findCostTarget(u, numNodes - 1);
            mcdsGraph.connect(mcdsGraph[index2], mcdsGraph[mcdsGraph.numNodes - 1], numNodes-1-u->id, cost);

        }


        // Return the new game graph
        
    }
    return mcdsGraph;
}



Edge* GameGraph::connect(Node* src, Node* dst, double normalCost, double honeypotCost) {
    edges.emplace_back(new Edge(numEdges++, src, dst, normalCost, honeypotCost));
    Edge* e = edges[numEdges - 1].get();
    Edge a = *e;
    edges2.emplace_back(a);

    src->out.push_back(e);
    dst->in.push_back(e);

    return e;
}

double GameGraph::findCostTarget(Node* n, int target)
{
    for (const auto& p : n->out) {
        if (p->dst->id == target) {
            return p->honeypotCost;
        }
    }
    return n->out[0]->honeypotCost + findCostTarget(n->out[0]->dst, target);

}

Node* GameGraph::operator[](unsigned int idx) {
    return &nodes[idx];
}
const Node* GameGraph::operator[](unsigned int idx) const {
    return &nodes[idx];
}