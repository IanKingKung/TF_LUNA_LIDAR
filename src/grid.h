#ifndef GRID_H
#define GRID_H

#include <vector>
#include "coordinate.h"
#include <cstdint>

class GRID {
    public: 
        int width;
        int height;
        float resolution;   //size of each cell in cm 
        std::vector<uint8_t> map_data; //1D array to represent 2D grid, 0 = free, 1 = occupied

        GRID(int w, int h, float res) : width(w), height(h), resolution(res), map_data(w * h, 0) {}

        // Set the value of a cell in the grid
        void set_cell(int x, int y, uint8_t value);

        //return the value of a cell in the grid
        uint8_t get_cell(int x, int y) const;

        //Marks a physical point (in meters relative to the sensor at 0,0) as occupied
        void mark_occupied_point(const Point2D& pt);

};



#endif // GRID_H