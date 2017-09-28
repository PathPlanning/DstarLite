#ifndef LOCALMAP_H
#define LOCALMAP_H

#include "map.h"

class LocalMap : public Map
{
public:
    LocalMap();
    ~LocalMap();

    bool lineOfSight(Cell a1, Cell a2, const Map &map, bool cutcorners);
    bool GetLocalMap(const Map &_map, Cell start, double radius);
    Changes UpdateMap(const Map &_map, Cell nstart, double radius);

};

#endif // LOCALMAP_H
