/**
 * @file positions.h
 * @brief Predefined robot arm positions and servo target definitions
 * @author Timo Berendsen, Benjamin Aarsen
 * @date 2025-10-08
 */

#pragma once

#include "RobotArmDriver.hpp"
#include <map>
#include <optional>

/// @brief Default speed for servo movements (in servo units)
constexpr int DEFAULT_SPEED = 200;

/// @brief Faster speed for servo movements (in servo units)
constexpr int FASTER_SPEED = 400;

/**
 * @struct ServoTarget
 * @brief Represents a target position for a single servo motor
 */
struct ServoTarget {
    int id;                        ///< Servo ID/channel number
    int position;                  ///< Target position in servo units
    int speed;                     ///< Movement speed in servo units
    std::optional<int> extra;      ///< Optional extra parameter
};

/// @brief Type alias for a collection of servo targets representing an arm position
using ArmPosition = std::vector<ServoTarget>;

/**
 * @enum Position
 * @brief Enumeration of predefined robot arm positions
 */
enum class Position {
    Park,        ///< Parked position - safe storage position
    StraightUp,  ///< Straight up position - vertical alignment
    Ready        ///< Ready position - prepared for operation
};

/**
 * @brief Map of predefined robot arm positions to their corresponding servo commands
 * 
 * This map contains three predefined positions:
 * - Park: Safe storage position for the robot arm
 * - StraightUp: Vertical alignment position 
 * - Ready: Operational ready position
 * 
 * Each position defines servo commands for channels 1-5 with specific:
 * - Channel number (1-5)
 * - Position value (servo-specific units)
 * - Movement speed (DEFAULT_SPEED or FASTER_SPEED)
 * - Optional time parameter (currently unused - std::nullopt)
 */
const std::map<Position, RobotArmDriver::MultiServoCommand> positions = {
    {Position::Park, {
        {1, 2200, DEFAULT_SPEED, std::nullopt},  ///< Servo 1: Park position
        {2, 2000, DEFAULT_SPEED, std::nullopt},  ///< Servo 2: Park position
        {3, 800, DEFAULT_SPEED, std::nullopt},   ///< Servo 3: Park position
        {4, 1450, DEFAULT_SPEED, std::nullopt},  ///< Servo 4: Park position
        {5, 1900, DEFAULT_SPEED, std::nullopt}   ///< Servo 5: Park position
    }},
    {Position::StraightUp, {
        {1, 1500, DEFAULT_SPEED, std::nullopt},  ///< Servo 1: Straight up position
        {2, 600, DEFAULT_SPEED, std::nullopt},   ///< Servo 2: Straight up position
        {3, 1450, DEFAULT_SPEED, std::nullopt},  ///< Servo 3: Straight up position
        {4, 1450, DEFAULT_SPEED, std::nullopt},  ///< Servo 4: Straight up position
        {5, 2500, DEFAULT_SPEED, std::nullopt}   ///< Servo 5: Straight up position
    }},
    {Position::Ready, {
        {1, 1850, FASTER_SPEED, std::nullopt},   ///< Servo 1: Ready position
        {2, 1650, FASTER_SPEED, std::nullopt},   ///< Servo 2: Ready position
        {3, 1300, FASTER_SPEED, std::nullopt},   ///< Servo 3: Ready position
        {4, 1500, FASTER_SPEED, std::nullopt},   ///< Servo 4: Ready position
        {5, 1000, FASTER_SPEED, std::nullopt}    ///< Servo 5: Ready position
    }}
};