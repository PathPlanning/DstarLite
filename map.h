#ifndef MAP_H
#define	MAP_H
#include <iostream>
#include "node.h"
#include "environmentoptions.h"
#include "gl_const.h"
#include <list>
#include "structures.h"
#include "tinyxml2.h"
#include <sstream>
#include <string>
#include <algorithm>
#include <random>


class Map
{
    public:
        Map();
        Map(const Map& orig);
        ~Map();

        Changes DamageTheMap(std::list<Node> path);
        bool GetMap(const char *name);
        bool CellIsTraversable (Cell curr) const;
        bool CellOnGrid (Cell curr) const;
        bool CellIsObstacle(Cell cur) const;
        bool CellIsNeighbor(Cell next, Cell curr) const;

        bool Cut(Cell next, Cell current) const;
        bool Squeeze(Cell next, Cell current) const;

        int * operator [] (int i);
        const int * operator [] (int i) const;

        void PrintPath(std::list<Node> path);

        int     height, width;
        Cell    start;
        Cell    goal;
        EnvironmentOptions algorithm_info;
        double  CellSize;


    protected:

        int **  Grid;

        void BuildGrid();

        std::string filename;

};

#endif
