#ifndef GRID_H
#define GRID_H

#include <vector>
#include "coordinate.h"
#include <cstdint>
#include <cstdlib>  
#include <SDL2/SDL.h>
#include <cmath>
#include <queue>

//represent a 2D grid, and responsible to marking cells as occupied or free 

class GRID {
    public: 
        int width;
        int height;
        float resolution;   //size of each cell in cm 

        //keep track of all cells that are an obstacle/occupied
        std::vector<uint8_t> occupied_list; 

        //1D array to represent 2D grid
        //-1 = unknown, 0 = occupied/obstacle, 1 = free
        std::vector<uint8_t> map_data; 

        GRID(int w, int h, float res) : width(w), height(h), resolution(res), map_data(w * h, 2) {}

        // Set the value of a cell in the grid
        void set_cell(int x, int y, uint8_t value);

        //return the location/index of the cell in the grid
        int get_cell_index(int x, int y) const;

        //return the value of a cell in the grid
        int get_cell_value(int x, int y) const;

        //Raytrace from the sensor at (0,0) to the hit point and mark all cells along the
        //path as free
        void raytrace_and_mark(const POINT2D& hit_pt);

        //change the values for all free cells to indicate its distance from obstacles/occupied cells
        //lower values we should tend to avoid (cell is closer to an obstacle)
        void update_clearances() {};

        //render grid to SDL2 renderer
        void render(SDL_Renderer* renderer, int cell_size) const;

        //determine if a point is within the bounds of the grid
        bool is_within_bounds(const POINT2D& point) const;

        //determine if a point is an obstacle
        bool is_obstacle(const POINT2D& point) const;

};



#endif // GRID_H