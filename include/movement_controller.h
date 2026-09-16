#ifndef MOVEMENT_CONTROLLER_H
#define MOVEMENT_CONTROLLER_H

#include "coordinate.h"
#include "serial_port.h"
#include "command.h"

class MovementController {
    public:
        MovementController::MovementController(float start_x, float start_y, float start_theta)
            : current_x(start_x), current_y(start_y), theta(start_theta), 
            current_speed(0.5f), is_scanning(false), is_moving(false) {
            
            // Open the serial port once when the controller is created
            if (!serial.open_port(port_path, 115200)) {
                std::cerr << "Failed to connect on port path: " << port_path << std::endl;
            }
        }

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
        void set_is_scanning(bool scanning);
        void set_is_moving(bool moving); 

    private:
        //robot's current position and orientation
        float current_x;
        float current_y;
        float theta; // in degrees - 0 degrees facing up on the map, 90 facing right, 180 facing down, etc.

        //track internal state of the robot
        float current_speed;
        bool is_scanning;
        bool is_moving;

        //serial port for sending movement commands to the robot
        std::string port_path = "/dev/cu.usbserial-2130"; // path to serial monitor
        SerialPort serial;
};

#endif // MOVEMENT_CONTROLLER_H