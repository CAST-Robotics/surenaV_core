#pragma once
#ifndef FINGER_CONTROL_H
#define FINGER_CONTROL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <mutex>
#include "json.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>

using namespace std;
using json = nlohmann::json;

// Finger Motor Data Structure: 29 + (0-5: target positions, 6-7: controls)

// Hand selection enum
enum class HandSelection {
    RIGHT_HAND = 0,
    LEFT_HAND = 1,
};

struct FingerScenario {
    std::string name;
    std::vector<uint8_t> target_positions;   // 0-255
    uint8_t control_data;
    
    FingerScenario() : control_data(0) {
        target_positions.resize(6, 0);
    }
};

class FingerControl {
public:
    FingerControl(ros::NodeHandle* nh);
    
    // Scenario management
    bool executeScenario(const std::string& name, HandSelection hand = HandSelection::RIGHT_HAND);
    
    // Direct control
    bool setDirectControl(const std::vector<uint8_t>& target_positions,
                          uint8_t control_data, 
                          HandSelection hand = HandSelection::RIGHT_HAND);
    
    // Utility method to convert string to HandSelection
    static HandSelection stringToHandSelection(const std::string& hand_str);

    const std::vector<double>& getFingerCommands() const;
    
private:
    ros::NodeHandle* nh_;
    ros::Publisher publish_trigger_pub_;

    std::map<std::string, FingerScenario> scenarios_;
    std::mutex scenarios_mutex_;
    
    vector<double> finger_commands_ = std::vector<double>(8, 0.0);
};

#endif // FINGER_CONTROL_H
