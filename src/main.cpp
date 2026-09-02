#include <iostream>
#include <vector>
#include <SDL2/SDL.h>
#include "grid.h"
#include "coordinate.h"

int main(int argc, char* argv[]) {
    
    int grid_width = 100; // number of cells in x direction
    int grid_height = 100; // number of cells in y direction
    float cell_resolution = 0.05f; // 5 cm per cell
    int cell_pixel_size = 6; // each grid cell is rendered as 6x6 square

    GRID map(grid_width, grid_height, cell_resolution);

    //LiDAR scan data
    int total_steps = 2048; // total steps in one full rotation of the LiDAR
    for (int step = 0; step < total_steps; ++step) {
        float simulated_distance = 2.0f; // 2 meters wall distance
        Point2D hit = steps_to_cartesian(simulated_distance, step, total_steps);
        map.raytrace_and_mark(hit);
    }

    //initialize SDL2
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    int window_width = grid_width * cell_pixel_size;
    int window_height = grid_height * cell_pixel_size;
    SDL_Window* window = SDL_CreateWindow("2D LiDAR Occupancy Grid", 
                                          SDL_WINDOWPOS_CENTERED, 
                                          SDL_WINDOWPOS_CENTERED, 
                                          window_width, window_height, 
                                          SDL_WINDOW_SHOWN);

    //initialize SDL2 rednerer
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    // 4. Application Loop
    bool running = true;
    SDL_Event e;

    while (running) {
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                running = false;
            }
        }

        // Clear Screen
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        // Render Grid
        map.render(renderer, cell_pixel_size);

        // Present Rendered Frame
        SDL_RenderPresent(renderer);
    }

    // Clean up
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}