#include "FingerControl.h"

// Finger Motor Data Structure: 29 + (0-5: target positions, 6-7: controls)
// 0: close , 255: open

FingerControl::FingerControl(ros::NodeHandle* nh) : nh_(nh) {
    // Initialize ROS communication
    publish_trigger_pub_ = nh->advertise<std_msgs::Empty>("robot_manager/publish_trigger", 1);
    
    // Load scenarios from JSON file
    std::string scenarios_path = ros::package::getPath("hand_planner") + "/config/finger_scenarios.json";
    std::ifstream fr(scenarios_path);
    json scenarios_json = json::parse(fr);
    
    for (auto& [name, scenario_data] : scenarios_json["scenarios"].items()) {
        FingerScenario scenario;
        scenario.name = name;
        scenario.target_positions = scenario_data["target_positions"].get<std::vector<uint8_t>>();
        scenario.control_data = scenario_data["control_data"].get<uint8_t>();
        scenarios_[name] = scenario;
    }
}

bool FingerControl::executeScenario(const std::string& name, HandSelection hand) {
    std::lock_guard<std::mutex> lock(scenarios_mutex_);
    auto it = scenarios_.find(name);
    if (it == scenarios_.end()) {
        ROS_ERROR("Scenario '%s' not found", name.c_str());
        return false;
    }
    const FingerScenario& scenario = it->second;
    for (int i = 0; i < 8; ++i) {
        finger_commands_[i] = 0;
    }
    // Fill positions (0-5)
    for (int i = 0; i < 6; ++i) {
        finger_commands_[i] = scenario.target_positions[i];
    }
    // Set controls based on hand selection
    if (hand == HandSelection::RIGHT_HAND) {
        finger_commands_[6] = scenario.control_data;  // Right hand control
    }
    if (hand == HandSelection::LEFT_HAND) {
        finger_commands_[7] = scenario.control_data;  // Left hand control
    }
    // Publish 2 messages
    ros::Rate rate(200);
    for (int i=0; i<2; i++) {
        publish_trigger_pub_.publish(std_msgs::Empty());
        rate.sleep();
        for (int i = 0; i < 8; ++i) {
            finger_commands_[i] = 0;
        }
    }
    return true;
}

bool FingerControl::setDirectControl(const std::vector<uint8_t>& target_positions,
                                    uint8_t control_data,
                                    HandSelection hand) {  
    // Validate input sizes
    if (target_positions.size() != 6) {
        ROS_ERROR("Invalid target positions size: %zu (expected: 6)", target_positions.size());
        return false;
    }    
    for (int i = 0; i < 8; ++i) {
        finger_commands_[i] = 0;
    }
    // Fill target positions (0-5)
    for (int i = 0; i < 6; ++i) {
        finger_commands_[i] = target_positions[i];
    }
    // Set controls based on hand selection
    if (hand == HandSelection::RIGHT_HAND) {
        finger_commands_[6] = control_data;  // Right hand control
    }
    if (hand == HandSelection::LEFT_HAND) {
        finger_commands_[7] = control_data;  // Left hand control
    }
    // Publish 2 messages
    ros::Rate rate(200);
    for (int i=0; i<2; i++) {
        publish_trigger_pub_.publish(std_msgs::Empty());
        rate.sleep();
        for (int i = 0; i < 8; ++i) {
            finger_commands_[i] = 0;
        }
    }
    return true;
}

HandSelection FingerControl::stringToHandSelection(const std::string& hand_str) {
    // Convert to lowercase for case-insensitive comparison
    std::string lower_hand = hand_str;
    std::transform(lower_hand.begin(), lower_hand.end(), lower_hand.begin(), ::tolower);
    
    if (lower_hand == "right") {
        return HandSelection::RIGHT_HAND;
    } else if (lower_hand == "left") {
        return HandSelection::LEFT_HAND;
    } else {
        ROS_WARN("Invalid hand selection: '%s', defaulting to RIGHT_HAND", hand_str.c_str());
        return HandSelection::RIGHT_HAND;  // Default to right hand
    }
}

const std::vector<double>& FingerControl::getFingerCommands() const {
    return finger_commands_;
}