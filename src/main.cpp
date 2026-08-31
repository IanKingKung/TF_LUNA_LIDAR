#include <iostream>
#include "coordinate.h"

int main() {
    float distance = 2.0f; // 2 meters
    int current_step = 400;
    int total_steps = 3200; // 200 steps * 16 microsteps

    POINT2D pt = steps_to_cartesian(distance, current_step, total_steps);

    std::cout << "Cartesian Point: X = " << pt.x << ", Y = " << pt.y << std::endl;
    return 0;
}