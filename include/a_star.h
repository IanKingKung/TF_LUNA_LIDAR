#ifndef A_STAR_H
#define A_STAR_H

#include "coordinate.h"
#include "grid.h"
#include <vector>
#include <cmath>
#include <algorithm> //for reverse
#include <queue> 
#include <unordered_map>

struct Node {   //define custom node structure for A* algorithm
    POINT2D position;
    float g_score;
    float h_score;
    float f_score;

    bool operator>(const Node& other) const {
        return f_score > other.f_score;
    }
    bool operator<(const Node& other) const {
        return f_score < other.f_score;
    }
    bool operator==(const Node& other) const {
        return position.x == other.position.x && position.y == other.position.y;
    }
    bool operator != (const Node& other) const {
        return position.x != other.position.x || position.y != other.position.y;
    }
};

//return a path of Points to get from start to goal using A*
std::vector<POINT2D> a_star(const POINT2D& start, const POINT2D& goal, const GRID& grid);

#endif // A_STAR_H