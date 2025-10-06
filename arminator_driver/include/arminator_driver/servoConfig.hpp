#pragma once
#include <map>
#include <utility>
#include <string>

// These will be filled by parsing the config file
extern std::map<int, int> SERVO_OFFSETS;
extern std::map<int, std::pair<int, int>> SERVO_LIMITS;

/**
 * Load servo configuration from a file.
 */
bool load_servo_config(const std::string& path);