#include "movement_controller.h"

void MovementController::follow_path(const std::vector<POINT2D>& path, float cell_size) {
    //TODO
}

void MovementController::move_forward_one_cell(float cell_size) {
    //TODO
}

void MovementController::rotate(float degree) {
    //TODO
}

void MovementController::scan(float degree) {
    //TODO
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
bool MovementController::set_is_scanning(bool scanning) {
    is_scanning = scanning;
}
bool MovementController::set_is_moving(bool moving) {
    is_moving = moving;
}