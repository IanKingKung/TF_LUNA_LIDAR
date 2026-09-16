#include "movement_controller.h"

void MovementController::follow_path(const std::vector<POINT2D>& path, float cell_size) {
    //0 degrees = up, 90 degrees = right, 180 degrees = down, 270 degrees = left

    for (const auto& point: path) {
        // Calculate the angle needed to turn to face the next point
        float standard_angle = atan2(point.y - current_y, point.x - current_x) * 180.0f / M_PI; // Convert to degrees
        float target_angle = 90.0f - standard_angle; // Adjust to match the robot's coordinate system (atan2 0 degrees = right, etc)

        
        //calculate angle difference & normalize angle_difference to be within [-180, 180]
        float angle_difference = target_angle - theta;
        while (angle_difference > 180.0f) angle_difference -= 360.0f;
        while (angle_difference < -180.0f) angle_difference += 360.0f;

        rotate(angle_difference);
        move_forward_one_cell(cell_size); 
        update_position(point.x, point.y, target_angle);
    }
}

void MovementController::move_forward_one_cell(float cell_size) {
    MovementCommand cmd;
    cmd.cmd = CommandType::MOVE;
    cmd.speed = static_cast<int16_t>(current_speed * 100); // Convert m/s to cm/s
    cmd.distance = static_cast<int16_t>(cell_size * 100); // Convert meters to centimeters
    cmd.angle = 0; // No rotation for straight movement

    // Use send_command (or write_command, depending on your SerialPort class definition)
    if (!serial.send_command(cmd)) {
        std::cerr << "Failed to send movement command!" << std::endl;
    }
}

void MovementController::rotate(float degree) {
    MovementCommand cmd;
    cmd.cmd = CommandType::ROTATE;
    cmd.speed = static_cast<int16_t>(current_speed * 100); // Convert m/s to cm/s
    cmd.distance = 0; // No linear movement for rotation
    cmd.angle = static_cast<int16_t>(degree); // Set the rotation angle

    // Use send_command (or write_command, depending on your SerialPort class definition)
    if (!serial.send_command(cmd)) {
        std::cerr << "Failed to send rotation command!" << std::endl;
    }
}

void MovementController::scan(float degree) {
    MovementCommand cmd;
    cmd.cmd = CommandType::SCAN;
    cmd.speed = static_cast<int16_t>(current_speed * 100); // Convert m/s to cm/s
    cmd.distance = 0; // No linear movement for scanning
    cmd.angle = static_cast<int16_t>(degree); // Set the scan angle

    // Use send_command (or write_command, depending on your SerialPort class definition)
    if (!serial.send_command(cmd)) {
        std::cerr << "Failed to send scan command!" << std::endl;
    }
}

//***************************************************************/
//***************************************************************/
//                  Status Functions
//***************************************************************/  
//***************************************************************/   
void MovementController::update_position(float new_x, float new_y, float new_theta) {
    current_x = new_x;
    current_y = new_y;
    theta = new_theta;
}
void MovementController::set_speed(float speed) {
    current_speed = speed;
}
float MovementController::get_current_x() const {
    return current_x;
}
float MovementController::get_current_y() const {
    return current_y;
}
float MovementController::get_current_theta() const {
    return theta;
}
float MovementController::get_current_speed() const {
    return current_speed;
}
bool MovementController::get_is_scanning() const {
    return is_scanning;
}
bool MovementController::get_is_moving() const {
    return is_moving;
}
void MovementController::set_is_scanning(bool scanning) {
    is_scanning = scanning;
}
void MovementController::set_is_moving(bool moving) {
    is_moving = moving;
}