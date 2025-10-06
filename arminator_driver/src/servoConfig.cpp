#include "arminator_driver/servoConfig.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>

std::map<int, int> SERVO_OFFSETS;
std::map<int, std::pair<int, int>> SERVO_LIMITS;

bool load_servo_config(const std::string& path) {
    std::ifstream infile(path);

    if (!infile) {
        std::cerr << "Failed to open config file: " << path << "\n";
        throw std::runtime_error("Config file not found"); //We use exceptions :D
        return false;
    }

    std::string line;
    enum Section { NONE, OFFSETS, LIMITS };
    Section current = NONE;

    while (std::getline(infile, line)) {
        boost::algorithm::trim(line);
        if (line.empty() || line[0] == '#') continue;

        if (line == "[Offsets]") {
            current = OFFSETS;
            continue;
        }
        if (line == "[Limits]") {
            current = LIMITS;
            continue;
        }

        if (current == OFFSETS) {
            // Format: channel=offset
            size_t eq = line.find('=');
            if (eq == std::string::npos) continue;
            int channel = std::stoi(line.substr(0, eq));
            int offset = std::stoi(line.substr(eq + 1));
            SERVO_OFFSETS[channel] = offset;
        } else if (current == LIMITS) {
            // Format: channel=min,max
            size_t eq = line.find('=');
            if (eq == std::string::npos) continue;
            int channel = std::stoi(line.substr(0, eq));
            std::string vals = line.substr(eq + 1);
            size_t comma = vals.find(',');
            if (comma == std::string::npos) continue;
            int min = std::stoi(vals.substr(0, comma));
            int max = std::stoi(vals.substr(comma + 1));
            SERVO_LIMITS[channel] = {min, max};
        }
    }
    return true;
}