#ifndef MAP_H
#define	MAP_H
#include <iostream>
#include "node.h"
#include "environmentoptions.h"
#include "gl_const.h"
#include <list>
#include "tinyxml2.h"
#include "searchresult.h"
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

        bool GetMap(const char *name);
        bool CellIsTraversable (Cell curr) const;
        bool CellOnGrid (Cell curr) const;
        bool CellIsObstacle(Cell cur) const;

        int * operator [] (int i);
        const int * operator [] (int i) const;

        void PrintPath(std::list<Node> path);

        int     height, width;
        Cell    start;
        Cell    goal;
        double  CellSize;


    protected:

        int **  Grid;

        void BuildGrid();

        std::string filename;

};

#endif
