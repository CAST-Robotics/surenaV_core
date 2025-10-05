#include "FingerControl.h"

uint8_t global_finger_trigger = 0;
// Finger message with 17 elements: 6 positions + 6 pressure + 3 PID + 1 right trigger + 1 left trigger  

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
        scenario.pressure_limits = scenario_data["pressure_limits"].get<std::vector<uint8_t>>();
        scenario.pid_kp = scenario_data["pid_kp"].get<uint8_t>();
        scenario.pid_ki = scenario_data["pid_ki"].get<uint8_t>();
        scenario.pid_kd = scenario_data["pid_kd"].get<uint8_t>();
        
        scenarios_[name] = scenario;
    }
    ROS_INFO("Finger Control initialized");
}

bool FingerControl::executeScenario(const std::string& name, HandSelection hand) {
    std::lock_guard<std::mutex> lock(scenarios_mutex_);
    
    auto it = scenarios_.find(name);
    if (it == scenarios_.end()) {
        ROS_ERROR("Scenario '%s' not found", name.c_str());
        return false;
    }
    
    const FingerScenario& scenario = it->second;
    std::string hand_str = (hand == HandSelection::RIGHT_HAND) ? "right" : (hand == HandSelection::LEFT_HAND) ? "left" : "both";
    ROS_INFO("Executing scenario: %s for %s hand(s)", scenario.name.c_str(), hand_str.c_str());
    
    // Fill positions (indices 0-5)
    for (int i = 0; i < 6; ++i) {
        finger_commands_[i] = scenario.target_positions[i];
    }
    
    // Fill pressure limits (indices 6-11)
    for (int i = 0; i < 6; ++i) {
        finger_commands_[i + 6] = scenario.pressure_limits[i];
    }
    
    // Fill PID values (indices 12-14)
    finger_commands_[12] = scenario.pid_kp;
    finger_commands_[13] = scenario.pid_ki;
    finger_commands_[14] = scenario.pid_kd;
    
    // Set triggers based on hand selection (indices 15-16)
    if (hand == HandSelection::RIGHT_HAND || hand == HandSelection::BOTH_HANDS) {
        finger_commands_[15] = global_finger_trigger;  // Right hand trigger
    }
    if (hand == HandSelection::LEFT_HAND || hand == HandSelection::BOTH_HANDS) {
        finger_commands_[16] = global_finger_trigger;  // Left hand trigger
    }
    
    // Publish message for 1 seconds
    ros::Rate rate(200);
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 1.0) {
        publish_trigger_pub_.publish(std_msgs::Empty());
        rate.sleep();
    }
    
    ROS_INFO("Scenario '%s' executed successfully for %s hand(s)", scenario.name.c_str(), hand_str.c_str());
    return true;
}

bool FingerControl::setDirectControl(const std::vector<uint8_t>& positions,
                                    const std::vector<uint8_t>& limits,
                                    uint8_t kp, uint8_t ki, uint8_t kd,
                                    HandSelection hand) {
    ROS_INFO("Setting direct finger control parameters");
    
    // Validate input sizes
    if (positions.size() != 6) {
        ROS_ERROR("Invalid positions size: %zu (expected: 6)", positions.size());
        return false;
    }
    if (limits.size() != 6) {
        ROS_ERROR("Invalid pressure limits size: %zu (expected: 6)", limits.size());
        return false;
    }
    
    std::string hand_str = (hand == HandSelection::RIGHT_HAND) ? "right" : (hand == HandSelection::LEFT_HAND) ? "left" : "both";
    ROS_INFO("Executing direct control for %s hand(s)", hand_str.c_str());
    
    // Fill positions (indices 0-5)
    for (int i = 0; i < 6; ++i) {
        finger_commands_[i] = positions[i];
    }
    
    // Fill pressure limits (indices 6-11)
    for (int i = 0; i < 6; ++i) {
        finger_commands_[i + 6] = limits[i];
    }
    
    // Fill PID values (indices 12-14)
    finger_commands_[12] = kp;
    finger_commands_[13] = ki;
    finger_commands_[14] = kd;
    
    // Set triggers based on hand selection (indices 15-16)
    if (hand == HandSelection::RIGHT_HAND || hand == HandSelection::BOTH_HANDS) {
        finger_commands_[15] = global_finger_trigger;  // Right hand trigger
    }
    if (hand == HandSelection::LEFT_HAND || hand == HandSelection::BOTH_HANDS) {
        finger_commands_[16] = global_finger_trigger;  // Left hand trigger
    }

    // Publish message for 1 seconds
    ros::Rate rate(200);
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 1.0) {
        publish_trigger_pub_.publish(std_msgs::Empty());
        rate.sleep();
    }
    
    ROS_INFO("Direct finger control executed successfully for %s hand(s)", hand_str.c_str());
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
    } else if (lower_hand == "both") {
        return HandSelection::BOTH_HANDS;
    } else {
        ROS_WARN("Invalid hand selection: '%s', defaulting to RIGHT_HAND", hand_str.c_str());
        return HandSelection::RIGHT_HAND;  // Default to right hand
    }
}

const std::vector<double>& FingerControl::getFingerCommands() const {
    return finger_commands_;
}