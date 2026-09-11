#ifndef A_STAR_H
#define A_STAR_H

#include "coordinate.h"
#include "grid.h"
#include <vector>

//return a path of Points to get from start to goal using A*
std::vector<POINT2D> a_star(const POINT2D& start, const POINT2D& goal, const GRID& grid);

#endif // A_STAR_H