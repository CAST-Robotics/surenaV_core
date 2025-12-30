#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include "ros/ros.h"
#include <memory>
#include <yaml-cpp/yaml.h>
#include "ros/callback_queue.h"
#include <std_msgs/Empty.h> 

// Include managers from library packages
#include "HandManager.h"
#include "GaitManager.h"

// Include our new service definition
#include "robot_manager/ExecuteScenario.h"
#include <robot_manager/JointCommand.h>

class RobotManager {
public:
    RobotManager(ros::NodeHandle *n);

private:
    bool simulation;

    // --- ROS Communication ---
    ros::NodeHandle* nh_;
    ros::ServiceServer execute_scenario_service_;
    ros::Subscriber publish_trigger_sub_;
    ros::ServiceServer joint_command_service_;

    ros::Publisher combined_motor_pub_;     
    ros::Publisher combined_gazebo_pub_; 

    // --- Pointers to Specialized Managers ---
    std::unique_ptr<HandManager> hand_manager_;
    std::unique_ptr<GaitManager> gait_manager_;

    // message objects 
    std_msgs::Int32MultiArray combined_motor_command_msg_;
    std_msgs::Float64MultiArray combined_gazebo_command_msg_;

    // --- Member to hold the parsed scenario data ---
    YAML::Node scenarios_config_;

    // --- Service Handlers ---
    bool execute_scenario_callback(robot_manager::ExecuteScenario::Request &req, robot_manager::ExecuteScenario::Response &res);
    bool jointCommandCallback(robot_manager::JointCommand::Request &req, robot_manager::JointCommand::Response &res); 
    // --- Helper Function ---
    void load_scenarios_from_file();
    bool execute_step(const YAML::Node& step);
    void publishTriggerCallback(const std_msgs::Empty::ConstPtr& msg);
    void publishCombinedMotorCommands();
    int32_t map_range(double val, double min_val, double max_val, double min_cmd, double max_cmd);
};

#endif // ROBOT_MANAGER_H