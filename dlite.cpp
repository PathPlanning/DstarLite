#include "dlite.h"


inline int vertex(Cell item, int size) {
    return item.x * size + item.y;
}

inline bool CutOrSqueeze(Node* to, Node* from) {
    if (to->point.x == from->point.x) {
        return (to->parent->point == Cell(from->point.x - 1, from->point.y) ||
                to->parent->point == Cell(from->point.x + 1, from->point.y));
    }
    else if (to->point.y == from->point.y) {
        return (to->parent->point == Cell(from->point.x, from->point.y - 1) ||
                to->parent->point == Cell(from->point.x, from->point.y + 1));
    }
    else return false;
}

double Dlite::GetCost(Cell from, Cell to, LocalMap &map) const {
    if (from.x == to.x || from.y == to.y) return 1;
    else return CN_SQRT_TWO;
}

Key Dlite::CalculateKey(const Node& vertex, LocalMap &map) {
    Key res(std::min(vertex.g, vertex.rhs + hweight * heuristic(vertex.point, map.start, map) + Km), std::min(vertex.g, vertex.rhs));
    return res;
}

Dlite::Dlite() {}
Dlite::Dlite(double rad, double HW) { radius = rad; hweight = HW; }
Dlite::~Dlite()
{
    if (!NODES.empty()) NODES.erase(NODES.begin(), NODES.end());
}

SearchResult Dlite::FindThePath(LocalMap &map, const Map& const_map, EnvironmentOptions options)
{
    opt = options;
    std::chrono::time_point<std::chrono::system_clock> tstart, end;
    tstart = std::chrono::system_clock::now();
    number_of_steps = 0;
    current_result.pathlength = 0;
    Initialize(map);
    last = start;

    if(!ComputeShortestPath(map))
        std::cout << "OOOPS\n";
    else
        std::cout << "Done\n";
    std::cout << current_result.pathlength <<std::endl;
    while(start->point != goal->point) {
        path.push_back(*start);
        Node min_val = GetMinPredecessor(start, map);
        if (!min_val.parent) {
            OPEN.remove_if(start);
            current_result.pathfound = false;
            current_result.pathlength = 0;
            return current_result;
        } else {
            current_result.pathlength += GetCost(start->point, min_val.parent->point, map);
            start = min_val.parent;
            //std::cout << "changed: "
            //          << neighbor->point << ' ' << neighbor->parent->point << std::endl;
            UpdateVertex(start, map);
        }

        Changes changes = map.UpdateMap(const_map, start->point, radius);
        if (!changes.cleared.empty() && !changes.occupied.empty()) {
            Km += heuristic(last->point, start->point, map);
            last = start;
        }
        for (auto dam : changes.occupied) {
            if (NODES.count(vertex(dam, map.height))) {
                Node* d = &(NODES.find(vertex(dam, map.height))->second);
                OPEN.remove_if(d);
                for (auto neighbor: GetSurroundings(d, map)) {
                    //std::cout << "n: " << neighbor->point << std::endl;
                    if (neighbor->point != map.goal && (neighbor->parent->point == dam || CutOrSqueeze(neighbor, d))) {
                        Node min_val = GetMinPredecessor(neighbor, map);
                        if (!min_val.parent) {
                            OPEN.remove_if(neighbor);
                            if(neighbor->point == start->point) {
                                current_result.pathfound = false;
                                current_result.pathlength = 0;
                                return current_result;
                            }
                        } else {
                            neighbor->rhs = min_val.rhs;
                            neighbor->parent = min_val.parent;
                            std::cout << "changed: "
                                      << neighbor->point << ' ' << neighbor->parent->point << std::endl;
                            UpdateVertex(neighbor, map);
                        }
                    }
                }
            }
        }
        for (auto cleared : changes.cleared) {
           Node new_node(cleared);
           new_node.rhs = std::numeric_limits<double>::infinity();
           new_node.g = std::numeric_limits<double>::infinity();
           NODES[vertex(cleared, map.height)] = new_node;
           Node * cl = &(NODES.find(vertex(cleared, map.height))->second);
           Node min_val = GetMinPredecessor(cl, map);
           if (min_val.parent) {
               cl->rhs = min_val.rhs;
               cl->parent = min_val.parent;
               cl->g = min_val.parent->g + GetCost(cl->point, min_val.parent->point, map);
               UpdateVertex(cl, map);
           } else {
               break;
           }
           for (auto neighbor : GetSuccessors(cl, map)) {
               if (neighbor->rhs > cl->g + GetCost(neighbor->point, cl->point, map)) {
                   neighbor->parent = cl;
                   neighbor->rhs = cl->g + GetCost(neighbor->point, cl->point, map);
                   UpdateVertex(neighbor, map);
               }
               if (neighbor->point.x == cl->point.x || neighbor->point.y == cl->point.y) {
                   Node min_val = GetMinPredecessor(neighbor, map);
                   if (!min_val.parent) {
                       OPEN.remove_if(neighbor);
                       if(neighbor->point == start->point) {
                           current_result.pathfound = false;
                           current_result.pathlength = 0;
                           return current_result;
                       }
                   } else {
                       neighbor->rhs = min_val.rhs;
                       neighbor->parent = min_val.parent;
                       //std::cout << "changed: "
                       //          << neighbor->point << ' ' << neighbor->parent->point << std::endl;
                       UpdateVertex(neighbor, map);
                   }
               }
           }
        }
        if(ComputeShortestPath(map)){
            std::cout << "ALL OK\n";
        } else
            std::cout << "NOT OK\n";
    }
    end = std::chrono::system_clock::now();
    current_result.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - tstart).count()) / 1000000000;
    map.PrintPath(path);
    current_result.lppath = &path;
    if (current_result.pathfound) {
        makeSecondaryPath();
        current_result.hppath = &hpath;
    }
    //for (auto elem: path) std::cout << elem->point << " ";
    //std::cout << std::endl;
    return current_result;
}



void Dlite::Initialize(LocalMap &map)
{
    Km = 0;
    Node start_node = Node(map.start);
    Node goal_node = Node(map.goal);
    goal_node.rhs = 0;
    goal_node.g = std::numeric_limits<double>::infinity();
    goal_node.key = CalculateKey(goal_node, map);
    NODES[vertex(map.goal, map.height)] = goal_node;
    goal = &(NODES.find(vertex(map.goal, map.height))->second);

    start_node.g = std::numeric_limits<double>::infinity();
    start_node.rhs = std::numeric_limits<double>::infinity();

    NODES[vertex(map.start, map.height)] = start_node;
    start = &(NODES.find(vertex(map.start, map.height))->second);

    OPEN.resize(map.height);
    OPEN.put(goal);
}

void Dlite::UpdateVertex(Node* u, LocalMap &map)
{
    if (!(u->IsConsistent())) {
        u->key = CalculateKey(*u, map);
        //std::cout << "key: "<< u->key.k1 << " " << u->key.k2 << " " << u->point << std::endl;
        OPEN.put(u); //check if it is already there
    } else {
        OPEN.remove_if(u);
    }
}

bool Dlite::ComputeShortestPath(LocalMap &map)
{
    while (OPEN.top_key_less_than(CalculateKey(*start, map)) || start->rhs != start->g) {
        ++number_of_steps;
        Node* current = OPEN.get();\
        Key old_key = current->key;
        Key new_key = CalculateKey(*current, map);
        if (old_key < new_key) {
            current->key = new_key;
            OPEN.put(current);
        } else if (current->g > current->rhs) {
            current->g = current->rhs;
            OPEN.pop();
            for (auto elem : GetSuccessors(current, map)) {
                if (elem->point != map.goal && elem->rhs > current->g + GetCost(elem->point, current->point, map)) {
                    elem->parent = current;
                    elem->rhs = current->g + GetCost(elem->point, current->point, map);
                }
                UpdateVertex(elem, map); // !!!<-apparently OK
            }
        } else {
            //double old_g = current->g;
            current->g = std::numeric_limits<double>::infinity();
            std::vector<Node* > succ = GetSuccessors(current, map);
            succ.push_back(current);
            for (auto elem : succ) {
                if (elem->point != map.goal && elem->parent->point == current->point) {
                    Node min_val = GetMinPredecessor(elem, map);
                    elem->rhs = min_val.rhs;
                    if(min_val.parent)
                        elem->parent = min_val.parent;
                }
                UpdateVertex(elem, map);
            }
        }
        if(current->parent) {
            std::cout << current->point << "g " << current->g << " rhs" << current->rhs <<
                  current->parent->point << std::endl;
        }     //std::cout << OPEN.top_key().k1 << std::endl;
        //OPEN.print_elements();

    }
    if (start->rhs != std::numeric_limits<double>::infinity()) {
        current_result.pathfound = true;
        current_result.numberofsteps = number_of_steps;
        current_result.nodescreated = NODES.size();
        std::cout << start->rhs << std::endl;
        MakePrimaryPath(start);
        std::cout << opt.cutcorners << std::endl;
        //current_result.lppath = &path;
        map.PrintPath(curpath);
        //for (auto elem : curpath) path.push_back(elem);
        return true;
    } else {
        current_result.pathfound = false;
        current_result.pathlength = 0;
    }
    return false;
}

Node Dlite::GetMinPredecessor(Node* current, LocalMap &map) {
    Node* min;
    std::vector<Node *> all_neighbors;
    for(auto elem : FindNeighbors(current, map)) {
        if(!NODES.count(vertex(elem.point, map.height))) { //if vertex wasn't previously examined
            elem.g =  std::numeric_limits<double>::infinity();
            elem.rhs = std::numeric_limits<double>::infinity();
            NODES[vertex(elem.point, map.height)] = elem;
            all_neighbors.push_back(&(NODES.find(vertex(elem.point, map.height))->second));
        } else {
            all_neighbors.push_back(&(NODES.find(vertex(elem.point, map.height))->second));
        }
    }
    Node min_node;
    if (!all_neighbors.empty()) {
        min = (all_neighbors.front());
        min_node = *min;
        min_node.rhs = std::numeric_limits<double>::infinity();
        min_node.parent = min;
        for (auto n: all_neighbors) {
            if (min_node.rhs > n->g + GetCost(n->point, current->point, map)) {
                min_node.rhs = n->g + GetCost(n->point, current->point, map);
                min_node.parent = n;
            }
        }
    } else {
        min_node.parent = nullptr;
    }
    return min_node;
}

std::vector<Node* > Dlite::GetSuccessors(Node* current, LocalMap &map) {
    std::vector<Node*> result;
    for(auto elem : FindNeighbors(current, map)) {
        if(!NODES.count(vertex(elem.point, map.height))) { //if vertex wasn't previously examined
            elem.g =  std::numeric_limits<double>::infinity();
            elem.rhs = std::numeric_limits<double>::infinity();
            NODES[vertex(elem.point, map.height)] = elem;
            result.push_back(&(NODES.find(vertex(elem.point, map.height))->second));
        } else {
            result.push_back(&(NODES.find(vertex(elem.point, map.height))->second));
        }
    }
    return result;
}

std::list<Node> Dlite::FindNeighbors(Node* n, LocalMap &map) const {
    Node newNode;
    Cell curNode = n->point;
    std::list<Node> successors;
    for (int i = -1; i <= +1; i++)
        for (int j = -1; j <= +1; j++)
            if ((i != 0 || j != 0) && map.CellOnGrid(Cell(curNode.x + j, curNode.y + i)) &&
                    (map.CellIsTraversable(Cell(curNode.x + j, curNode.y + i)))) {
                if (i != 0 && j != 0) {
                    if (!opt.allowdiagonal)
                        continue;
                    else if (!opt.cutcorners) {
                        if (map.CellIsObstacle(Cell(curNode.x + j, curNode.y)) ||
                                map.CellIsObstacle(Cell(curNode.x, curNode.y + i)))
                            continue;
                    }
                    else if (!opt.allowsqueeze) {
                        if (map.CellIsObstacle(Cell( curNode.x + j, curNode.y)) &&
                                map.CellIsObstacle(Cell( curNode.x, curNode.y + i)))
                            continue;
                    }
                }
                newNode = Node(Cell(curNode.x + j, curNode.y + i), n);
                successors.push_front(newNode);
            }
    return successors;
    /*int x = n->point.x;
    int y = n->point.y;
    if(!opt.allowdiagonal) {
        for (int k = y - 1; k <= y + 1; ++k) {
            for (int l = x - 1; l <= x + 1; ++l) {
                if (!(k == y && l == x) && map.CellIsNeighbor(Cell(l, k), n->point)) {
                    result.push_back(Node(Cell(l, k), n));
                }
            }
        }
    } else {
        for (int k = x - 1; k <= x + 1; ++k)
            if (k != x && map.CellOnGrid(Cell(k, y)) && map.CellIsTraversable(Cell(k, y)))
                result.push_back(Node(Cell(k, y), n));
        for (int l = y - 1; l <= y + 1; ++l)
            if (l != y && map.CellOnGrid(Cell(x, l)) && map.CellIsTraversable(Cell(x, l)))
                result.push_back(Node(Cell(x, l), n));
    }
    return result;*/
}

std::list<Node*> Dlite::GetSurroundings(Node *current, LocalMap &map) {
    std::list<Node> result1;
    int x = current->point.x;
    int y = current->point.y;
    if(opt.allowdiagonal) {
        for (int k = y - 1; k <= y + 1; ++k) {
            for (int l = x - 1; l <= x + 1; ++l) {
                if (!(k == y && l == x) && map.CellOnGrid(Cell(l, k)) && map.CellIsTraversable(Cell(l, k))) {
                    result1.push_front(Node(Cell(l, k)));
                }
            }
        }
    } else {
        for (int k = x - 1; k <= x + 1; ++k)
            if (k != x && map.CellOnGrid(Cell(k, y)) && map.CellIsTraversable(Cell(k, y)))
                result1.push_front(Node(Cell(k, y)));
        for (int l = y - 1; l <= y + 1; ++l)
            if (l != y && map.CellOnGrid(Cell(x, l)) && map.CellIsTraversable(Cell(x, l)))
                result1.push_front(Node(Cell(x, l)));
    }
    std::list<Node*> result;
    for(auto elem : result1) {
        if(!NODES.count(vertex(elem.point, map.height))) { //if vertex wasn't previously examined
            continue;
        } else {
            result.push_back(&(NODES.find(vertex(elem.point, map.height))->second));
        }
    }
    return result;
}

void Dlite::MakePrimaryPath(Node *curNode)
{
    curpath.clear();
    Node current = *curNode;
    while (current.parent) {
        curpath.push_front(current);
        current = *current.parent;
    }
    curpath.push_back(current);
}

void Dlite::makeSecondaryPath()
{
    std::list<Node>::const_iterator iter = path.begin();
    int curI, curJ, nextI, nextJ, moveI, moveJ;
    hpath.push_back(*iter);
    while (iter != --path.end()) {
        curI = iter->point.y;
        curJ = iter->point.x;
        ++iter;
        nextI = iter->point.y;
        nextJ = iter->point.x;
        moveI = nextI - curI;
        moveJ = nextJ - curJ;
        ++iter;
        if ((iter->point.y - nextI) != moveI || (iter->point.x - nextJ) != moveJ)
            hpath.push_back(*(--iter));
        else
            --iter;
    }
}
