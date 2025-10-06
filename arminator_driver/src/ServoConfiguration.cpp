#include "arminator_driver/ServoConfiguration.hpp"
#include <iostream>
#include <stdexcept>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string.hpp>

int ServoConfiguration::getOffset(int channel) const {
    auto it = offsets.find(channel);
    return (it != offsets.end()) ? it->second : 0;
}

std::pair<int, int> ServoConfiguration::getLimits(int channel) const {
    auto it = limits.find(channel);
    return (it != limits.end()) ? it->second : std::make_pair(500, 2500);  // Default generic limits
}

std::string ServoConfiguration::getSerialPort() const {
    return serial_port.find("/dev/") == 0 ? serial_port : "/dev/serial/by-id/" + serial_port;
}

ServoConfiguration ServoConfiguration::loadFromFile(const std::string& path) {
    ServoConfiguration config;
    boost::property_tree::ptree pt;

    try {
        // Load the .ini file
        boost::property_tree::read_ini(path, pt);

        // Parse Config section
        try {
            auto config_section = pt.get_child("Config");
            
            config.serial_port = config_section.get<std::string>("serial_port", config.serial_port);
            config.baud_rate = config_section.get<int>("baud_rate", config.baud_rate);
            config.arm_name = config_section.get<std::string>("arm_name", config.arm_name);
            config.servo_count = config_section.get<int>("servo_count", config.servo_count);
            
        } catch (const boost::property_tree::ptree_bad_path&) {
            std::cerr << "Warning: No Config section found, using defaults" << std::endl;
        }

        // Parse Offsets section
        try {
            auto offsets_section = pt.get_child("Offsets");
            for (const auto& pair : offsets_section) {
                try {
                    int channel = std::stoi(pair.first);
                    int offset = pair.second.get_value<int>();
                    config.offsets[channel] = offset;
                } catch (const std::exception& e) {
                    std::cerr << "Warning: Invalid offset entry '" << pair.first 
                              << "=" << pair.second.get_value<std::string>() 
                              << "': " << e.what() << std::endl;
                }
            }
        } catch (const boost::property_tree::ptree_bad_path&) {
            std::cerr << "Warning: No Offsets section found" << std::endl;
        }

        // Parse Limits section
        try {
            auto limits_section = pt.get_child("Limits");
            for (const auto& pair : limits_section) {
                try {
                    int channel = std::stoi(pair.first);
                    std::string limits_str = pair.second.get_value<std::string>();
                    
                    // Parse min,max format
                    size_t comma = limits_str.find(',');
                    if (comma == std::string::npos) {
                        std::cerr << "Warning: Invalid limits format for channel " << channel 
                                  << ", expected 'min,max': " << limits_str << std::endl;
                        continue;
                    }
                    
                    std::string min_str = limits_str.substr(0, comma);
                    std::string max_str = limits_str.substr(comma + 1);
                    boost::algorithm::trim(min_str);
                    boost::algorithm::trim(max_str);
                    
                    int min = std::stoi(min_str);
                    int max = std::stoi(max_str);
                    
                    if (min >= max) {
                        std::cerr << "Warning: Invalid limits for channel " << channel 
                                  << ", min (" << min << ") >= max (" << max << ")" << std::endl;
                        continue;
                    }
                    
                    config.limits[channel] = {min, max};
                } catch (const std::exception& e) {
                    std::cerr << "Warning: Invalid limits entry '" << pair.first 
                              << "=" << pair.second.get_value<std::string>() 
                              << "': " << e.what() << std::endl;
                }
            }
        } catch (const boost::property_tree::ptree_bad_path&) {
            std::cerr << "Warning: No Limits section found" << std::endl;
        }

    } catch (const boost::property_tree::ini_parser_error& e) {
        throw std::runtime_error("Failed to parse INI file '" + path + "': " + e.what());
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load config file '" + path + "': " + e.what());
    }

    // Validate configuration
    if (config.serial_port.empty()) {
        std::cerr << "Warning: No serial port specified, using default" << std::endl;
    }

    // Log loaded configuration
    std::cout << "Loaded configuration for arm: " << config.arm_name << std::endl;
    std::cout << "Serial port: " << config.getSerialPort() << std::endl;
    std::cout << "Baud rate: " << config.baud_rate << std::endl;
    std::cout << "Servo count: " << config.servo_count << std::endl;
    std::cout << "Loaded " << config.offsets.size() << " offset(s) and " 
              << config.limits.size() << " limit(s)" << std::endl;

    return config;
}

