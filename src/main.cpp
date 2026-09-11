#include <iostream>
#include <SDL2/SDL.h>
#include "grid.h"
#include "coordinate.h"
#include "serial_port.h"

int main(int argc, char* argv[]) {
    //Initialize Grid (100x100 cells, 0.05m / 5cm per cell = 5m x 5m coverage)
    GRID map(100, 100, 0.05f);
    int total_steps = 2048;

    //Open Serial Connection (Replace with your actual serial port path)
    SerialPort serial;
    std::string port_path = "/dev/cu.usbserial-2130"; // path to serial monitor
    if (!serial.open_port(port_path, 115200)) {
        std::cerr << "Failed to connect to Arduino on " << port_path << std::endl;
        return 1;
    }

    //Initialize SDL Window & Renderer
    SDL_Init(SDL_INIT_VIDEO);
    int cell_pixel_size = 6;
    SDL_Window* window = SDL_CreateWindow("Live TF-Luna LiDAR Map", 
                                          SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 
                                          map.width * cell_pixel_size, map.height * cell_pixel_size, 
                                          SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    bool running = true;
    SDL_Event e;

    int live_step = 0;
    float live_distance_m = 0.0f;

    //Main Event & Visualization Loop
    while (running) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
        }

        // Drain serial buffer and update grid with incoming live points
        while (serial.read_scan_data(live_step, live_distance_m)) {
            // Filter out TF-Luna error/out-of-range readings (valid range ~0.1m to 8.0m)
            if (live_distance_m > 0.1f && live_distance_m < 8.0f) {
                POINT2D hit = steps_to_cartesian(live_distance_m, live_step, total_steps);
                map.raytrace_and_mark(hit);
            }
        }

        // Render updated map
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        map.render(renderer, cell_pixel_size);
        SDL_RenderPresent(renderer);
    }

    serial.close_port();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}