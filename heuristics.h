#ifndef HEURISTICS
#define HEURISTICS

#include "node.h"
#include "gl_const.h"
#include <cmath>

inline double manhattan_heuristic(const Cell& a, const Cell& b) {
  return abs(a.x - b.x) + abs(a.y - b.y);
}

inline double euclid_heuristic(const Cell& a, const Cell& b) {
    return sqrt(abs(a.x - b.x) * abs(a.x - b.x) + abs(a.y - b.y) * abs(a.y - b.y));
}

inline double octile_heuristic(const Cell& a, const Cell& b) {
    return (abs(abs(a.x - b.x) - abs(a.y - b.y)) + CN_SQRT_TWO * (std::min(abs(a.x - b.x), abs(a.y - b.y))));
}

inline double cheb_heuristic(const Cell& a, const Cell& b) {
    return std::max(abs(a.x - b.x), abs(a.y - b.y));
}

inline double heuristic(const Cell& a, const Cell& b, int metrictype) {
    if (metrictype == CN_SP_MT_EUCL) return euclid_heuristic(a, b);
    if (metrictype == CN_SP_MT_MANH) return manhattan_heuristic(a, b);
    if (metrictype == CN_SP_MT_DIAG) return octile_heuristic(a, b);
    if (metrictype == CN_SP_MT_CHEB) return cheb_heuristic(a, b);
    return 1;
}

#endif // HEURISTICS

