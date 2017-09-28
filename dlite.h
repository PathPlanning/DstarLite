#ifndef DLITE_H
#define DLITE_H

#include "localmap.h"
#include "openlist.h"
#include "searchresult.h"
#include "heuristics.h"
#include "environmentoptions.h"
#include <unordered_map>
#include <set>
#include <chrono>

class Dlite
{
public:
    Dlite();
    Dlite(double rad, double HW);
    ~Dlite(void);

    SearchResult FindThePath(LocalMap &map, const Map& const_map, EnvironmentOptions options);
    void MakePrimaryPath(Node* curNode);
    void makeSecondaryPath();

private:
    Node *start;
    Node *goal;
    Node *last;
    int number_of_steps;

    double Km;
    double radius;

    EnvironmentOptions opt;
    std::list<Node> curpath;
    std::list<Node> path;
    std::list<Node> hpath;

    double linecost;
    double hweight;
    Cell start_point;
    SearchResult current_result;
    OpenList OPEN;
    std::unordered_map<int, Node> NODES;

    void Initialize(LocalMap &map);
    void UpdateVertex(Node* u);
    bool ComputeShortestPath(LocalMap &map);
    double GetCost(Cell from, Cell to, LocalMap &map) const;
    Key CalculateKey(const Node &vertex);

    std::vector<Node *> GetSuccessors(Node *curr, LocalMap &map);
    std::list<Node *> GetSurroundings(Node *current, LocalMap &map);
    Node GetMinPredecessor(Node* curr, LocalMap &map);
    std::list<Node> FindNeighbors(Node* curr, LocalMap &map) const;
};


#endif // DLITE_H
