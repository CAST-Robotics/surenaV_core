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

extern uint8_t global_finger_trigger;
// Finger message with 17 elements: 6 positions + 6 pressure + 3 PID + 1 right trigger + 1 left trigger  

// Hand selection enum
enum class HandSelection {
    RIGHT_HAND = 0,
    LEFT_HAND = 1,
    BOTH_HANDS = 2
};

struct FingerScenario {
    std::string name;
    std::vector<uint8_t> target_positions;   // 0-255
    std::vector<uint8_t> pressure_limits;    // 0-255
    uint8_t pid_kp;                          // Single value (0-255)
    uint8_t pid_ki;                          // Single value (0-255)
    uint8_t pid_kd;                          // Single value (0-255)
    
    FingerScenario() : pid_kp(1), pid_ki(0), pid_kd(0) {
        target_positions.resize(6, 0);
        pressure_limits.resize(6, 0);
    }
};

class FingerControl {
public:
    FingerControl(ros::NodeHandle* nh);
    
    // Scenario management
    bool executeScenario(const std::string& name, HandSelection hand = HandSelection::RIGHT_HAND);
    
    // Direct control
    bool setDirectControl(const std::vector<uint8_t>& positions,
                         const std::vector<uint8_t>& limits,
                         uint8_t kp, uint8_t ki, uint8_t kd,
                         HandSelection hand = HandSelection::RIGHT_HAND);
    
    // Utility method to convert string to HandSelection
    static HandSelection stringToHandSelection(const std::string& hand_str);

    const std::vector<double>& getFingerCommands() const;
    
private:
    ros::NodeHandle* nh_;
    ros::Publisher publish_trigger_pub_;

    std::map<std::string, FingerScenario> scenarios_;
    std::mutex scenarios_mutex_;
    
    vector<double> finger_commands_ = std::vector<double>(17, 0.0);
};

#endif // FINGER_CONTROL_H
