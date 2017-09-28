#include "localmap.h"

inline double euclid_dist(Cell a, Cell b) {
    return sqrt(abs(a.x - b.x) * abs(a.x - b.x) + abs(a.y - b.y) * abs(a.y - b.y));
}

LocalMap::LocalMap() { Map(); }
LocalMap::~LocalMap() {}

bool LocalMap::lineOfSight(Cell a1, Cell a2, const Map &map, bool cutcorners)
{
    int i1 = a1.y;
    int j1 = a1.x;
    int i2 = a2.y;
    int j2 = a2.x;
    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);
    int error = 0;
    int i = i1 + step_i;
    int j = j1 + step_j;
    if(delta_i == 0) {
        for(; j != j2; j += step_j)
            if(map.CellIsObstacle(Cell(j, i)))
                return false;
        return true;
    }
    else if(delta_j == 0) {
        for(; i != i2; i += step_i)
            if(map.CellIsObstacle(Cell(j, i)))
                return false;
        return true;
    }
    if(cutcorners) {
        if (delta_i > delta_j) {
            for (; i != i2; i += step_i) {
                if (map.CellIsObstacle(Cell(j, i)))
                    return false;
                error += delta_j;
                if ((error << 1) > delta_i) {
                    if(((error << 1) - delta_j) < delta_i && map.CellIsObstacle(Cell(j, i+step_i)))
                            return false;
                    else if(((error << 1) - delta_j) > delta_i && map.CellIsObstacle(Cell(j+step_j, i)))
                            return false;
                    j += step_j;
                    error -= delta_i;
                }
            }
        }
        else {
            for (; j != j2; j += step_j) {
                if (map.CellIsObstacle(Cell(j, i)))
                    return false;
                error += delta_i;
                if ((error << 1) > delta_j) {
                    if(((error << 1) - delta_i) < delta_j && map.CellIsObstacle(Cell(j+step_j, i)))
                            return false;
                    else if(((error << 1) - delta_i) > delta_j && map.CellIsObstacle(Cell(j, i+step_i)))
                            return false;
                    i += step_i;
                    error -= delta_j;
                }
            }
        }

    }
    else {
        int sep_value = delta_i*delta_i + delta_j*delta_j;
        if(delta_i > delta_j) {
            for(; i != i2; i += step_i) {
                if(map.CellIsObstacle(Cell(j, i)))
                    return false;
                if(map.CellIsObstacle(Cell(j + step_j, i)))
                    return false;
                error += delta_j;
                if(error >= delta_i) {
                    if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                        if(map.CellIsObstacle(Cell(j, i + step_i)))
                            return false;
                    if((3*delta_i - ((error << 1) - delta_j))*(3*delta_i - ((error << 1) - delta_j)) < sep_value)
                        if(map.CellIsObstacle(Cell(j + 2*step_j, i)))
                            return false;
                    j += step_j;
                    error -= delta_i;
                }
            }
            if(map.CellIsObstacle(Cell(j, i)))
                return false;
        }
        else {
            for(; j != j2; j += step_j) {
                if(map.CellIsObstacle(Cell(j, i)))
                    return false;
                if(map.CellIsObstacle(Cell(j, i + step_i)))
                    return false;
                error += delta_i;
                if(error >= delta_j) {
                    if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < (delta_i*delta_i + delta_j*delta_j))
                        if(map.CellIsObstacle(Cell(j + step_j, i)))
                            return false;
                    if((3*delta_j - ((error << 1) - delta_i))*(3*delta_j - ((error << 1) - delta_i)) < (delta_i*delta_i + delta_j*delta_j))
                        if(map.CellIsObstacle(Cell(j, i + 2*step_i)))
                            return false;
                    i += step_i;
                    error -= delta_j;
                }
            }
            if(map.CellIsObstacle(Cell(j, i)))
                return false;
        }
    }
    return true;
}

bool LocalMap::GetLocalMap(const Map &_map, Cell nstart, double radius) {
    height = _map.height;
    width = _map.width;
    start = _map.start;
    goal = _map.goal;
    CellSize = _map.CellSize;
    algorithm_info = _map.algorithm_info;
    BuildGrid();
    for (int y = static_cast<int>(nstart.y - radius); y < static_cast<int>(nstart.y + radius); ++y) {
        for (int x = static_cast<int>(nstart.x - radius); x < static_cast<int>(nstart.x + radius); ++x) {
            if (CellOnGrid(Cell(x, y)) && euclid_dist(start, Cell(x,y)) < radius) {
                //if (lineOfSight(Cell(x, y), nstart, _map, _map.algorithm_info.cutcorners)) Grid[y][x] = _map[y][x];
                //else Grid[y][x] = CN_GC_NOOBS;
                Grid[y][x] = _map[y][x];
            }
        }
    }
    return true;
}

Changes LocalMap::UpdateMap(const Map& _map, Cell new_start, double radius) {
    Changes change;
    for (int y = static_cast<int>(new_start.y - radius); y < static_cast<int>(new_start.y + radius); ++y) {
        for (int x = static_cast<int>(new_start.x - radius); x < static_cast<int>(new_start.x + radius); ++x) {
            if (CellOnGrid(Cell(x, y)) && euclid_dist(new_start, Cell(x,y)) < radius) {
                /*if (lineOfSight(Cell(x, y), new_start, _map, _map.algorithm_info.cutcorners)) {
                    if (Grid[y][x] != _map[y][x]) {
                        if (_map[y][x] == CN_GC_NOOBS) change.cleared.push_back(Cell(x, y));
                        else change.occupied.push_back(Cell(x, y));
                        Grid[y][x] = _map[y][x];
                    }
                }*/
                if (Grid[y][x] != _map[y][x]) {
                    if (_map[y][x] == CN_GC_NOOBS) change.cleared.push_back(Cell(x, y));
                    else change.occupied.push_back(Cell(x, y));
                    Grid[y][x] = _map[y][x];
                }
            }
        }
    }
    /*for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            std::cout << Grid[i][j] << " ";
        }
        std::cout << std::endl;
    }*/
    return change;
}


