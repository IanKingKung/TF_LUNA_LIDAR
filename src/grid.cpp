#include "grid.h"

void GRID::set_cell(int x, int y, uint8_t value) {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        map_data[y * width + x] = value;
    }
}

uint8_t GRID::get_cell(int x, int y) const {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        return map_data[y * width + x];
    }
    return 0; // Return default value for out-of-bounds access
}

void Grid::mark_occupied_point(const Point2D& pt) {
    // Sensor sits at the center cell of the local grid
    int sensor_cell_x = width / 2;
    int sensor_cell_y = height / 2;

    // Convert meters from sensor (0,0) into grid cell offsets
    int grid_x = sensor_cell_x + static_cast<int>(std::round(pt.x / resolution));
    int grid_y = sensor_cell_y + static_cast<int>(std::round(pt.y / resolution));

    // Mark as occupied if within local grid bounds
    set_cell(grid_x, grid_y, 1);
}