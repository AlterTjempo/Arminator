#pragma once

#include <string>
#include <vector>
#include <memory>

namespace arminator {

/**
 * @brief Low-level serial communication interface for AL5D robot arm
 */
class SerialInterface {
public:
    /**
     * @brief Constructor
     * @param port Serial port device path
     * @param baud_rate Baud rate for communication
     */
    SerialInterface(const std::string& port, int baud_rate);
    
    /**
     * @brief Destructor
     */
    ~SerialInterface();
    
    /**
     * @brief Open serial connection
     * @return true if successful
     */
    bool open();
    
    /**
     * @brief Close serial connection
     */
    void close();
    
    /**
     * @brief Check if connection is open
     * @return true if connected
     */
    bool isOpen() const;
    
    /**
     * @brief Send command to robot
     * @param command Command string to send
     * @return true if sent successfully
     */
    bool sendCommand(const std::string& command);
    
    /**
     * @brief Read response from robot
     * @param timeout_ms Timeout in milliseconds
     * @return Response string
     */
    std::string readResponse(int timeout_ms = 1000);
    
    /**
     * @brief Send servo position command
     * @param servo_id Servo ID (0-5)
     * @param position Position value (500-2500 typically)
     * @param speed Speed value (0-1023)
     * @return true if sent successfully
     */
    bool sendServoCommand(int servo_id, int position, int speed = 512);
    
    /**
     * @brief Send multiple servo commands
     * @param servo_positions Vector of positions for each servo
     * @param speed Global speed for all servos
     * @return true if sent successfully
     */
    bool sendMultiServoCommand(const std::vector<int>& servo_positions, int speed = 512);
    
    /**
     * @brief Query servo position
     * @param servo_id Servo ID (0-5)
     * @return Current position or -1 if error
     */
    int queryServoPosition(int servo_id);
    
    /**
     * @brief Convert angle to servo pulse width
     * @param angle_degrees Angle in degrees
     * @param servo_id Servo ID for calibration
     * @return Pulse width value
     */
    static int angleToServoValue(double angle_degrees, int servo_id);
    
    /**
     * @brief Convert servo pulse width to angle
     * @param servo_value Pulse width value
     * @param servo_id Servo ID for calibration
     * @return Angle in degrees
     */
    static double servoValueToAngle(int servo_value, int servo_id);
    
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

} // namespace arminator