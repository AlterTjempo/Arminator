#pragma once
#include <map>
#include <utility>
#include <string>

struct ServoConfiguration {
    std::string serial_port;
    int baud_rate;
    std::string arm_name;
    int servo_count;
    std::map<int, int> offsets;
    std::map<int, std::pair<int, int>> limits;
    
    // Default constructor with sensible defaults
    ServoConfiguration() : 
        serial_port("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH06DKH3-if00-port0"),
        baud_rate(115200),
        arm_name("Unknown"),
        servo_count(6) {}
    
    // Get offset for a servo channel, returns 0 if not found
    int getOffset(int channel) const;
    
    // Get limits for a servo channel, returns default limits if not found
    std::pair<int, int> getLimits(int channel) const;
    
    // Get full serial port path
    std::string getSerialPort() const;
    
    // Load configuration from file
    static ServoConfiguration loadFromFile(const std::string& path);
};