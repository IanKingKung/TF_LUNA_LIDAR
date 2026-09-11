#ifndef MOVEMENT_CONTROLLER_H
#define MOVEMENT_CONTROLLER_H

#include "coordinate.h"
#include "serial_port.h"

class MovementController {
    public:
        MovementController(float start_x = 0.0, float start_y = 0.0, float start_theta = 0.0);  //constructor

        void follow_path(const std::vector<POINT2D>& path, float cell_size);  //follow a path of points in the grid
        void move_forward_one_cell(float cell_size);  //move forward one cell in the grid
        void rotate(float degree);  //rotate in degrees, positive = clockwise
        void scan(float degree);    //scan in degrees, 0 = straight ahead, 90 = right, -90 = left

        //status functions   
        void update_position(float new_x, float new_y, float new_theta);
        void set_speed(float speed);
        float get_current_x() const;
        float get_current_y() const;
        float get_current_theta() const;
        float get_current_speed() const;
        bool get_is_scanning() const;
        bool get_is_moving() const;

    private:
        //robot's current position and orientation
        float current_x;
        float current_y;
        float theta; // in degrees - 0 degrees facing up on the map

        //track internal state of the robot
        float current_speed;
        bool is_scanning;
        bool is_moving;

};

#endif // MOVEMENT_CONTROLLER_H