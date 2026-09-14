#include "serial_port.h"

bool SerialPort::open_port(const std::string& port_name, int baud_rate) {
    fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); //open port in read/write mode, no controlling terminal, non-blocking
    if (fd == -1) {
        std::cerr << "Error opening serial port: " << port_name << std::endl;
        return false;
    }

    // Set non-blocking read mode
    fcntl(fd, F_SETFL, FNDELAY);

    struct termios options;
    tcgetattr(fd, &options);

    // Map integer baud rates to termios constants
    speed_t speed = B115200;
    if (baud_rate == 9600) speed = B9600;
    else if (baud_rate == 57600) speed = B57600;
    else if (baud_rate == 115200) speed = B115200;

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, set local mode
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE; // Clear current data size setting
    options.c_cflag |= CS8; // 8 data bits

    // Raw input / disable canonical processing
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcflush(fd, TCIFLUSH); // Flush the input buffer
    tcsetattr(fd, TCSANOW, &options);
    return true;
}

bool SerialPort::read_distance(float& distance_out) {
    if (fd == -1) return false;

    // Read available bytes into incoming buffer
    uint8_t temp[64];
    ssize_t bytes_read = read(fd, temp, sizeof(temp));
    if (bytes_read > 0) {
        rx_buffer.insert(rx_buffer.end(), temp, temp + bytes_read);
    }

    // Packet structure expected: [0xAA] [4 bytes float] -> 5 bytes total
    while (rx_buffer.size() >= 5) {
        // Look for frame header byte (0xAA)
        if (rx_buffer[0] != 0xAA) {
            rx_buffer.erase(rx_buffer.begin()); // Drop invalid byte to realign
            continue;
        }

        // Extract float payload
        std::memcpy(&distance_out, &rx_buffer[1], sizeof(float));

        // Erase processed 5-byte packet
        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + 5);
        return true;
    }

    return false; // Waiting for complete frame
}

bool SerialPort::read_scan_data(int& step_out, float& distance_meters_out) {
    if (fd == -1) return false;

    uint8_t temp[64];
    ssize_t bytes_read = read(fd, temp, sizeof(temp));
    if (bytes_read > 0) {
        rx_buffer.insert(rx_buffer.end(), temp, temp + bytes_read);
    }

    // Packet size: 1 byte header + 2 bytes step + 2 bytes distance = 5 bytes total
    while (rx_buffer.size() >= 5) {
        // Sync to header byte
        if (rx_buffer[0] != 0xAA) {
            rx_buffer.erase(rx_buffer.begin()); // Drop invalid byte to realign
            continue;
        }

        uint16_t raw_step = 0;
        uint16_t raw_dist_cm = 0;

        std::memcpy(&raw_step, &rx_buffer[1], sizeof(uint16_t));
        std::memcpy(&raw_dist_cm, &rx_buffer[3], sizeof(uint16_t));

        step_out = static_cast<int>(raw_step);
        distance_meters_out = static_cast<float>(raw_dist_cm) / 100.0f; // Convert cm to meters

        // Erase processed packet from buffer
        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + 5);
        return true;
    }

    return false; // Waiting for full packet
}

void SerialPort::close_port() {
    if (fd != -1) {
        close(fd);
        fd = -1;
    }
}