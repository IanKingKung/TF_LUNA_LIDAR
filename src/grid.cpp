#include "grid.h"

void GRID::set_cell(int x, int y, uint8_t value) {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        map_data[y * width + x] = value;
    }
}

int GRID::get_cell_index(int x, int y) const {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        return y * width + x;
    }
    return -1;   //default value for if cell is not within grid
}

int GRID::get_cell_value(int x, int y) const {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        return map_data[y * width + x];
    }
    return -1; // Return default value for out-of-bounds access
}

void GRID::raytrace_and_mark(const POINT2D& hit_pt) {
    // Sensor sits at the center cell of the local grid
    int sensor_cell_x = width / 2;
    int sensor_cell_y = height / 2;

    // Convert meters from sensor (0,0) into grid cell offsets
    int hit_cell_x = sensor_cell_x + static_cast<int>(std::round(hit_pt.x / resolution));
    int hit_cell_y = sensor_cell_y + static_cast<int>(std::round(hit_pt.y / resolution));

    // Use Bresenham's line algorithm to mark cells along the ray as free
    int dx = std::abs(hit_cell_x - sensor_cell_x);
    int dy = std::abs(hit_cell_y - sensor_cell_y);
    int sx = (sensor_cell_x < hit_cell_x) ? 1 : -1; //determines direction of ray in x
    int sy = (sensor_cell_y < hit_cell_y) ? 1 : -1; //determines direction of ray in y
    int err = dx - dy;  //error term, used to determine whether to step in x or y direction

    int cur_x = sensor_cell_x;
    int cur_y = sensor_cell_y;

    while (cur_x != hit_cell_x || cur_y != hit_cell_y) {

        if (cur_x >= 0 && cur_x < width && cur_y >= 0 && cur_y < height) {
            set_cell(cur_x, cur_y, 1); // 1 = Free Space
        }

        int e2 = err * 2;   //scaled error
        if (e2 > -dy) { //compare scaled error to -dy as stepping in x reduces error by dy
            err -= dy;
            cur_x += sx;
        }
        if (e2 < dx) {  //compare scaled error to dx as stepping in y increases error by dx
            err += dx;
            cur_y += sy;
        }
    }

    if (hit_cell_x >= 0 && hit_cell_x < width && hit_cell_y >= 0 && hit_cell_y < height) {
        set_cell(hit_cell_x, hit_cell_y, 0); // Mark the hit point as occupied
        int occupied_index = get_cell_index(hit_cell_x, hit_cell_y);
        occupied_list.push_back(occupied_index);
    }
}

void GRID::update_clearances() {
    //grid is 5x5m or 500cm x 500cm ---> a score of 1 represents cell is 5cm away from obstacle

    //run BFS to calculate the clearances of the cells
    std::queue<int> q; 

    for (int obstacle_index : obstacle_list) {
        map_data[obstacle_index] = 0;
        q.push(obstacle_index);
    }

    // 8-way directional offsets for precise diagonal clearance propagation
    const int dx[] = {0, 0, 1, -1, 1, -1, 1, -1};
    const int dy[] = {1, -1, 0, 0, 1, 1, -1, -1};

    while (!q.empty()) {
        int current_cell_index = q.front();
        q.pop();

        int cur_x = current_cell_index % width;
        int cur_y = current_cell_index / width;
        uint8_t current_val = map_data[current_cell_index];

        // If the current clearance value has hit the maximum cap, stop propagating outward
        if (current_val >= 99) continue;

        // Check the 8 neighboring cells
        for (int i = 0; i < 8; ++i) {
            int next_x = cur_x + dx[i];
            int next_y = cur_y + dy[i];

            if (next_x >= 0 && next_x < width && next_y >= 0 && next_y < height) {
                int next_cell_index = next_y * width + next_x;
                uint8_t neighbor_val = map_data[next_cell_index];

                // Do not overwrite obstacles (0) or unknown space 
                uint8_t calculated_val = current_val + 1;
                if (calculated_val > 99) calculated_val = 99;

                // Propagate if neighbor is unassigned (e.g., raw free space marked as 1) 
                // or if we found a shorter path gradient than what it currently holds
                if (neighbor_val == 1 || (neighbor_val > calculated_val && neighbor_val != 0)) {
                    map_data[next_cell_index] = calculated_val;
                    q.push(next_cell_index);
                }
            }
        }
    }
}

void GRID::render(SDL_Renderer* renderer, int cell_pixel_size) const {
    
    //create a rectangle size of cell_size x cell_size
    SDL_Rect rect;
    rect.w = cell_pixel_size;
    rect.h = cell_pixel_size;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            uint8_t state = get_cell_value(x, y);

            // Choose color based on cell state
            if (state == 0) {
                // Occupied (Red)
                SDL_SetRenderDrawColor(renderer, 255, 50, 50, 255);
            } else if (state == 1) {
                // Free space (Light Gray / Off-white)
                SDL_SetRenderDrawColor(renderer, 220, 220, 220, 255);
            } else {
                // Unexplored/Unknown (Dark Gray)
                SDL_SetRenderDrawColor(renderer, 40, 40, 40, 255);
            }

            //set position of rectangle to x,y in the grid
            rect.x = x * cell_pixel_size;
            rect.y = y * cell_pixel_size;
            SDL_RenderFillRect(renderer, &rect);
        }
    }

    //Sensor location as a green dot at center
    int center_x = (width / 2) * cell_pixel_size;
    int center_y = (height / 2) * cell_pixel_size;
    rect.x = center_x;
    rect.y = center_y;
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); // Green
    SDL_RenderFillRect(renderer, &rect);
}

bool GRID::is_within_bounds(const POINT2D& point) const {
    int grid_x = width / 2 + static_cast<int>(std::round(point.x / resolution));    // Convert meters to grid cell coordinates
    int grid_y = height / 2 + static_cast<int>(std::round(point.y / resolution));
    return (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height);
}

bool GRID::is_obstacle(const POINT2D& point) const {
    int grid_x = width / 2 + static_cast<int>(std::round(point.x / resolution));    // Convert meters to grid cell coordinates
    int grid_y = height / 2 + static_cast<int>(std::round(point.y / resolution));
    if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
        return get_cell_value(grid_x, grid_y) == 0; // 0 = Occupied
    }
    return false; // Out of bounds is considered not an obstacle
}