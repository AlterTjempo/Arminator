#include "arminator/serial_interface.hpp"
#include <iostream>
#include <sstream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>

namespace arminator {

class SerialInterface::Impl {
public:
    Impl(const std::string& port, int baud_rate)
        : port_(port), baud_rate_(baud_rate), fd_(-1) {}
    
    ~Impl() {
        close();
    }
    
    bool open() {
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            std::cerr << "Failed to open serial port: " << port_ << std::endl;
            return false;
        }
        
        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            std::cerr << "Error getting terminal attributes" << std::endl;
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        
        // Configure serial port
        cfsetospeed(&tty, getBaudRate(baud_rate_));
        cfsetispeed(&tty, getBaudRate(baud_rate_));
        
        tty.c_cflag &= ~PARENB;        // No parity
        tty.c_cflag &= ~CSTOPB;        // One stop bit
        tty.c_cflag &= ~CSIZE;         // Clear size bits
        tty.c_cflag |= CS8;            // 8 data bits
        tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Enable reading, ignore control lines
        
        tty.c_lflag &= ~ICANON;        // Non-canonical mode
        tty.c_lflag &= ~ECHO;          // No echo
        tty.c_lflag &= ~ECHOE;         // No echo erase
        tty.c_lflag &= ~ECHONL;        // No echo newline
        tty.c_lflag &= ~ISIG;          // No signal processing
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        
        tty.c_oflag &= ~OPOST;         // No output processing
        tty.c_oflag &= ~ONLCR;         // No newline conversion
        
        tty.c_cc[VTIME] = 10;          // 1 second timeout
        tty.c_cc[VMIN] = 0;            // Non-blocking read
        
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting terminal attributes" << std::endl;
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        
        tcflush(fd_, TCIOFLUSH);
        return true;
    }
    
    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
    
    bool isOpen() const {
        return fd_ >= 0;
    }
    
    bool sendCommand(const std::string& command) {
        if (!isOpen()) {
            return false;
        }
        
        std::string cmd_with_newline = command + "\r\n";
        ssize_t bytes_written = write(fd_, cmd_with_newline.c_str(), cmd_with_newline.length());
        
        if (bytes_written < 0) {
            std::cerr << "Error writing to serial port" << std::endl;
            return false;
        }
        
        // Ensure data is transmitted
        tcdrain(fd_);
        return true;
    }
    
    std::string readResponse(int timeout_ms) {
        if (!isOpen()) {
            return "";
        }
        
        fd_set read_fds;
        struct timeval timeout;
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;
        
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);
        
        std::string response;
        char buffer[256];
        
        while (true) {
            int select_result = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
            
            if (select_result < 0) {
                std::cerr << "Error in select()" << std::endl;
                break;
            } else if (select_result == 0) {
                // Timeout
                break;
            }
            
            if (FD_ISSET(fd_, &read_fds)) {
                ssize_t bytes_read = read(fd_, buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';
                    response += buffer;
                    
                    // Check for end of response (e.g., newline)
                    if (response.find('\n') != std::string::npos) {
                        break;
                    }
                } else if (bytes_read < 0) {
                    std::cerr << "Error reading from serial port" << std::endl;
                    break;
                }
            }
        }
        
        return response;
    }
    
private:
    std::string port_;
    int baud_rate_;
    int fd_;
    
    speed_t getBaudRate(int baud) {
        switch (baud) {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            default: return B9600;
        }
    }
};

SerialInterface::SerialInterface(const std::string& port, int baud_rate)
    : pImpl(std::make_unique<Impl>(port, baud_rate)) {}

SerialInterface::~SerialInterface() = default;

bool SerialInterface::open() {
    return pImpl->open();
}

void SerialInterface::close() {
    pImpl->close();
}

bool SerialInterface::isOpen() const {
    return pImpl->isOpen();
}

bool SerialInterface::sendCommand(const std::string& command) {
    return pImpl->sendCommand(command);
}

std::string SerialInterface::readResponse(int timeout_ms) {
    return pImpl->readResponse(timeout_ms);
}

bool SerialInterface::sendServoCommand(int servo_id, int position, int speed) {
    if (servo_id < 0 || servo_id > 5) {
        return false;
    }
    
    std::ostringstream oss;
    oss << "#" << servo_id << "P" << position << "S" << speed;
    return sendCommand(oss.str());
}

bool SerialInterface::sendMultiServoCommand(const std::vector<int>& servo_positions, int speed) {
    if (servo_positions.size() > 6) {
        return false;
    }
    
    std::ostringstream oss;
    for (size_t i = 0; i < servo_positions.size(); ++i) {
        oss << "#" << i << "P" << servo_positions[i];
    }
    oss << "S" << speed << "T1000"; // 1 second execution time
    
    return sendCommand(oss.str());
}

int SerialInterface::queryServoPosition(int servo_id) {
    if (servo_id < 0 || servo_id > 5) {
        return -1;
    }
    
    std::ostringstream oss;
    oss << "#" << servo_id << "?";
    
    if (!sendCommand(oss.str())) {
        return -1;
    }
    
    std::string response = readResponse(1000);
    if (response.empty()) {
        return -1;
    }
    
    // Parse response to extract position
    try {
        return std::stoi(response);
    } catch (const std::exception&) {
        return -1;
    }
}

int SerialInterface::angleToServoValue(double angle_degrees, int servo_id) {
    // AL5D servo calibration values (adjust based on actual hardware)
    const int servo_min[] = {500, 500, 500, 500, 500, 500};
    const int servo_max[] = {2500, 2500, 2500, 2500, 2500, 2500};
    const double angle_min[] = {-90, -90, -90, -90, -90, 0};
    const double angle_max[] = {90, 90, 90, 90, 90, 180};
    
    if (servo_id < 0 || servo_id > 5) {
        return 1500; // Default center position
    }
    
    // Clamp angle to valid range
    double clamped_angle = std::max(angle_min[servo_id], 
                                   std::min(angle_max[servo_id], angle_degrees));
    
    // Linear interpolation
    double ratio = (clamped_angle - angle_min[servo_id]) / 
                   (angle_max[servo_id] - angle_min[servo_id]);
    
    return static_cast<int>(servo_min[servo_id] + 
                           ratio * (servo_max[servo_id] - servo_min[servo_id]));
}

double SerialInterface::servoValueToAngle(int servo_value, int servo_id) {
    // AL5D servo calibration values (adjust based on actual hardware)
    const int servo_min[] = {500, 500, 500, 500, 500, 500};
    const int servo_max[] = {2500, 2500, 2500, 2500, 2500, 2500};
    const double angle_min[] = {-90, -90, -90, -90, -90, 0};
    const double angle_max[] = {90, 90, 90, 90, 90, 180};
    
    if (servo_id < 0 || servo_id > 5) {
        return 0.0;
    }
    
    // Clamp servo value to valid range
    int clamped_value = std::max(servo_min[servo_id], 
                                std::min(servo_max[servo_id], servo_value));
    
    // Linear interpolation
    double ratio = static_cast<double>(clamped_value - servo_min[servo_id]) / 
                   static_cast<double>(servo_max[servo_id] - servo_min[servo_id]);
    
    return angle_min[servo_id] + ratio * (angle_max[servo_id] - angle_min[servo_id]);
}

} // namespace arminator