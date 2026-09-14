#ifndef COMMAND_H
#define COMMAND_H

#pragma once
#include <cstdint>

// Ensure the compiler doesn't add hidden padding bytes
#pragma pack(push, 1)

enum class CommandType : uint8_t {
    STOP = 0x01,
    MOVE = 0x02,
    SCAN = 0x03,
    ROTATE = 0x04,
};

struct MovementCommand {
    uint8_t sync_byte = 0xFF;
    CommandType cmd;
    int16_t speed;
    int16_t distance;
    int16_t angle;  //angle to turn as well as scan
};

#pragma pack(pop)
#endif // COMMAND_H