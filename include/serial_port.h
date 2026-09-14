#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cstdint>
#include <string>
#include <vector>

class SerialPort {
private:
    int fd = -1;
    std::vector<uint8_t> rx_buffer;

public:
    SerialPort() = default();

    bool open_port(const std::string& port_name, int baud_rate = 115200);
    bool read_distance(float& distance_out);
    bool read_scan_data(int& step_out, float& distance_meters_out);
    void close_port();

    template <typename T>
    bool send_command (const T& command) {
        if (fd == -1) return false;
        ssize_t bytes_written = write(fd, &command, sizeof(T));
        return bytes_written == static_cast<ssize_t>(sizeof(T));
    }

    ~SerialPort() { close_port(); }
};


#endif // SERIAL_PORT_H