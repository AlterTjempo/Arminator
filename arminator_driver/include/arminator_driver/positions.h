#pragma once

#include "RobotArmDriver.hpp"
#include <map>
#include <optional>

constexpr int DEFAULT_SPEED = 200;
constexpr int FASTER_SPEED = 400;

struct ServoTarget {
    int id;
    int position;
    int speed;
    std::optional<int> extra;
};

using ArmPosition = std::vector<ServoTarget>;

enum class Position {
    Park,
    StraightUp,
    Ready
};

const std::map<Position, RobotArmDriver::MultiServoCommand> positions = {
    {Position::Park, {
        {1, 2200, DEFAULT_SPEED, std::nullopt},
        {2, 2000, DEFAULT_SPEED, std::nullopt},
        {3, 800, DEFAULT_SPEED, std::nullopt},
        {4, 1450, DEFAULT_SPEED, std::nullopt},
        {5, 1900, DEFAULT_SPEED, std::nullopt}
    }},
    {Position::StraightUp, {
        {1, 1500, DEFAULT_SPEED, std::nullopt},
        {2, 600, DEFAULT_SPEED, std::nullopt},
        {3, 1450, DEFAULT_SPEED, std::nullopt},
        {4, 1450, DEFAULT_SPEED, std::nullopt},
        {5, 2500, DEFAULT_SPEED, std::nullopt}
    }},
    {Position::Ready, {
        {1, 1850, FASTER_SPEED, std::nullopt},
        {2, 1650, FASTER_SPEED, std::nullopt},
        {3, 1300, FASTER_SPEED, std::nullopt},
        {4, 1500, FASTER_SPEED, std::nullopt},
        {5, 1000, FASTER_SPEED, std::nullopt}
    }}
};