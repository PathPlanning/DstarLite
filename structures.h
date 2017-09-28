#ifndef STRUCTURES
#define STRUCTURES

#include "node.h"
#include <cmath>
#include <vector>
#include <functional>
#include <utility>
#include <iostream>
#include <chrono>

struct Changes {
    std::list<Cell> occupied;
    std::list<Cell> cleared;
};

struct Section {
    Node start;
    Node finish;
    double length;

    Section(Node s, Node f) : start(s), finish(f), length(sqrt(pow(f.point.x - s.point.x, 2) + pow(f.point.y - s.point.y, 2))) {}
};

struct Figure {
    double perimeter;
    double square;
    std::vector<Cell> cells;
    Figure() {}
    Figure(int p = 0, int s = 0, std::vector<Cell> c = {}) : perimeter(p), square(s), cells(c) {}
};

struct Algorithm {
    std::string searchtype;
    std::string metrictype;
    double hweight;
    std::string breakingties;
    double linecost;
    double diagonalcost;
    bool allowdiagonal;
    bool allowsqueeze;
    bool cutcorners;
    double loglevel;
    std::string logpath;
    std::string logfilename;

    Algorithm& operator=(const Algorithm& other) {
        searchtype = other.searchtype;
        metrictype = other.metrictype;
        hweight = other.hweight;
        breakingties = other.breakingties;
        linecost = other.linecost;
        diagonalcost = other.diagonalcost;
        allowdiagonal = other.allowdiagonal;
        allowsqueeze = other.allowsqueeze;
        cutcorners = other.cutcorners;
        loglevel = other.loglevel;
        logpath = other.logpath;
        logfilename = other.logfilename;
        return *this;
    }
};


#endif // STRUCTURES

