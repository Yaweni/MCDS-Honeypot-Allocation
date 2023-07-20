#ifndef STOCHASTICHONEYPOTGAME_GAMEGRAPH_H
#define STOCHASTICHONEYPOTGAME_GAMEGRAPH_H


#include <vector>
#include <memory>

class Edge;

class Node {
public:
    unsigned int id;

    std::vector<Edge*> out;
    std::vector<Edge*> in;

    Node(unsigned int id);
};

class Edge {
public:
    unsigned int id;

    Node* src;
    Node* dst;

    double normalCost;
    double honeypotCost;

    Edge(unsigned int id, Node* src, Node* dst, double normalCost, double honeypotCost);
};

class GameGraph {
public:
    unsigned int numNodes;
    unsigned int numEdges;

    std::vector<Node> nodes;
    std::vector<std::unique_ptr<Edge>> edges;
    std::vector<Edge> edges2;
    const Node* target;

    GameGraph(unsigned int numNodes);
    GameGraph findMCDS();
    Edge* connect(Node* src, Node* dst, double normalCost, double honeypotCost);
    double findCostTarget(Node* n, int target);
    Node* operator[](unsigned int idx);
    const Node* operator[](unsigned int idx) const;
};


#endif //STOCHASTICHONEYPOTGAME_GAMEGRAPH_H
#pragma once
