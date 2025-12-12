#include "RobotManager.h"

// --- CONSTRUCTOR ---
RobotManager::RobotManager(ros::NodeHandle *n) : 
    nh_(n),
    simulation(true)
{
    hand_manager_ = std::make_unique<HandManager>(nh_);
    gait_manager_ = std::make_unique<GaitManager>(nh_);

    load_scenarios_from_file();

    // ROS Communication Setup
    execute_scenario_service_ = nh_->advertiseService("execute_scenario_srv", &RobotManager::execute_scenario_callback, this);
    combined_motor_pub_       = nh_->advertise<std_msgs::Int32MultiArray>("jointdata/qc", 100);
    combined_gazebo_pub_      = nh_->advertise<std_msgs::Float64MultiArray>("/joint_angles_gazebo", 100);
    publish_trigger_sub_      = nh_->subscribe("robot_manager/publish_trigger", 1, &RobotManager::publishTriggerCallback, this);
}

// --- YAML FILE LOADER ---
void RobotManager::load_scenarios_from_file() {
    std::string file_path = ros::package::getPath("robot_manager") + "/config/scenarios.yaml";
    try {
        scenarios_config_ = YAML::LoadFile(file_path);
        YAML::Node scenarios_node = scenarios_config_["scenarios"];
        ROS_INFO("Successfully loaded scenarios. Available scenarios:");
        for (const auto& scenario : scenarios_node) {
            // scenario.first is the key (the scenario name) as a YAML::Node
            ROS_INFO("  - %s", scenario.first.as<std::string>().c_str());
        }
    } catch (const YAML::Exception& e) {
        ROS_FATAL("Failed to load or parse scenarios.yaml: %s", e.what());
    }
}

// --- MAIN SERVICE HANDLER ---
bool RobotManager::execute_scenario_callback(robot_manager::ExecuteScenario::Request &req, robot_manager::ExecuteScenario::Response &res) {
    ROS_INFO("Executing scenario: %s", req.scenario_name.c_str());
    YAML::Node steps;
    try {
        steps = scenarios_config_["scenarios"][req.scenario_name];
        if (!steps || !steps.IsSequence()) { throw std::runtime_error("is not a valid sequence."); }
    } catch (const std::exception& e) {
        res.success = false;
        res.message = "Scenario '" + req.scenario_name + "' error: " + e.what();
        ROS_ERROR("%s", res.message.c_str());
        return true;
    }

    for (int i = 0; i < steps.size(); ++i) {
        ROS_INFO("Executing step %d of %d...", i + 1, (int)steps.size());
        if (!execute_step(steps[i])) {
            res.success = false;
            res.message = "Failed at step " + std::to_string(i + 1) + ". Aborting scenario.";
            ROS_ERROR("%s", res.message.c_str());
            return true;
        }
    }

    res.success = true;
    res.message = "Scenario '" + req.scenario_name + "' completed successfully.";
    ROS_INFO("%s", res.message.c_str());
    return true;
}

// --- HELPER TO EXECUTE A SINGLE STEP ---
bool RobotManager::execute_step(const YAML::Node& step) {
    std::string service_name = step["service"].as<std::string>();
    YAML::Node params = step["params"];

    if (service_name == "/set_target_class_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::SetTargetClass>(service_name);
        hand_planner::SetTargetClass srv;
        srv.request.class_name = params["class_name"].as<std::string>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.class_id != -1;

    } else if (service_name == "/head_track_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::head_track>(service_name);
        hand_planner::head_track srv;
        srv.request.duration_seconds = params["duration_seconds"].as<double>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.success;

    } else if (service_name == "/move_hand_single_srv") {
        if (!params["mode"] || !params["ee_ini_pos"] || !params["scen_count"] || !params["t_total"] || !params["scenario"]) {
            ROS_ERROR("One or more required parameters are missing for %s", service_name.c_str()); return false;
        }
        ros::ServiceClient client = nh_->serviceClient<hand_planner::move_hand_single>(service_name);
        hand_planner::move_hand_single srv;
        srv.request.mode = params["mode"].as<std::string>();
        srv.request.ee_ini_pos = params["ee_ini_pos"].as<std::string>();
        srv.request.scen_count = params["scen_count"].as<int>();
        srv.request.t_total = params["t_total"].as<int>();
        srv.request.scenario = params["scenario"].as<std::vector<std::string>>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return true;

    } else if (service_name == "/move_hand_both_srv") {
        if (!params["scenarioR"] || !params["ee_ini_posR"] || !params["scenR_count"] || !params["scenarioL"] || !params["ee_ini_posL"] || !params["scenL_count"] || !params["t_total"]) {
            ROS_ERROR("One or more required parameters are missing for %s", service_name.c_str()); return false;
        }
        ros::ServiceClient client = nh_->serviceClient<hand_planner::move_hand_both>(service_name);
        hand_planner::move_hand_both srv;
        srv.request.ee_ini_posR = params["ee_ini_posR"].as<std::string>();
        srv.request.scenarioR = params["scenarioR"].as<std::vector<std::string>>();
        srv.request.scenR_count = params["scenR_count"].as<int>();
        srv.request.ee_ini_posL = params["ee_ini_posL"].as<std::string>();
        srv.request.scenarioL = params["scenarioL"].as<std::vector<std::string>>();
        srv.request.scenL_count = params["scenL_count"].as<int>();
        srv.request.t_total = params["t_total"].as<int>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return true;

    } else if (service_name == "/walk_service") {
        if (!params["alpha"] || !params["t_double_support"] || !params["t_step"] || !params["step_length"] || !params["step_width"] || !params["COM_height"] || !params["step_count"] ||
            !params["ankle_height"] || !params["dt"] || !params["theta"] || !params["step_height"] || !params["com_offset"] || !params["is_config"]) {
            ROS_ERROR("One or more required parameters are missing for %s", service_name.c_str()); return false;
        }
        ros::ServiceClient client = nh_->serviceClient<gait_planner::Trajectory>(service_name);
        gait_planner::Trajectory srv;
        srv.request.alpha = params["alpha"].as<double>();
        srv.request.t_double_support = params["t_double_support"].as<double>();
        srv.request.t_step = params["t_step"].as<double>();
        srv.request.step_length = params["step_length"].as<double>();
        srv.request.step_width = params["step_width"].as<double>();
        srv.request.COM_height = params["COM_height"].as<double>();
        srv.request.step_count = params["step_count"].as<int>();
        srv.request.ankle_height = params["ankle_height"].as<double>();
        srv.request.dt = params["dt"].as<double>();
        srv.request.theta = params["theta"].as<double>();
        srv.request.step_height = params["step_height"].as<double>();
        srv.request.com_offset = params["com_offset"].as<double>();
        srv.request.is_config = params["is_config"].as<bool>();
        srv.request.hand_swing_angle = params["hand_swing_angle"].as<double>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.result;
    
    } else if (service_name == "/grip_online_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::gripOnline>(service_name);
        hand_planner::gripOnline srv;
        srv.request.start = params["start"].as<std::string>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return !srv.response.finish.empty();

    } else if (service_name == "/home_service" || service_name == "/keyboard_walk" || service_name == "/print_absolute") {
        ros::ServiceClient client = nh_->serviceClient<std_srvs::Empty>(service_name);
        std_srvs::Empty srv;
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return true;

    } else if (service_name == "/arm_back_to_home_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::arm_back_to_home>(service_name);
        hand_planner::arm_back_to_home srv;
        if (!client.call(srv)) { 
            ROS_ERROR("Service call to %s failed.", service_name.c_str()); 
            return false; 
        }
        return srv.response.success;

    } else if (service_name == "/keyboard_walk_seq") {
        ros::ServiceClient client = nh_->serviceClient<gait_planner::KeyboardWalkSeq>(service_name);
        gait_planner::KeyboardWalkSeq srv;
        srv.request.seq = params["seq"].as<std::string>();
        if (!client.call(srv)) { 
            ROS_ERROR("Service call to %s failed.", service_name.c_str()); 
            return false; 
        }
        return srv.response.success;

    } else if (service_name == "/finger_control_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::FingerControl>(service_name);
        hand_planner::FingerControl srv;
        srv.request.target_positions = params["target_positions"].as<std::vector<int64_t>>();
        srv.request.control_data = params["control_data"].as<int64_t>();
        srv.request.hand_selection = params["hand_selection"].as<std::string>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.success;

    } else if (service_name == "/finger_scenario_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::FingerScenario>(service_name);
        hand_planner::FingerScenario srv;
        srv.request.scenario_name = params["scenario_name"].as<std::string>();
        srv.request.hand_selection = params["hand_selection"].as<std::string>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.success;

    } else if (service_name == "/move_hand_general_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::MoveHandGeneral>(service_name);
        hand_planner::MoveHandGeneral srv;
        srv.request.commands = params["commands"].as<std::vector<std::string>>();
        srv.request.go_home_on_finish = params["go_home_on_finish"].as<bool>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.ok;

    } else if (service_name == "/move_hand_general_left_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::MoveHandGeneral>(service_name);
        hand_planner::MoveHandGeneral srv;
        srv.request.commands = params["commands"].as<std::vector<std::string>>();
        srv.request.go_home_on_finish = params["go_home_on_finish"].as<bool>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.ok;

    } else if (service_name == "/move_hand_keyboard_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::KeyboardJog>(service_name);
        hand_planner::KeyboardJog srv;
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.ok;

    } else if (service_name == "/move_hand_relative_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::PickAndMove>(service_name);
        hand_planner::PickAndMove srv;
        srv.request.axes = params["axes"].as<std::vector<std::string>>();
        srv.request.deltas = params["deltas"].as<std::vector<double>>();
        srv.request.durations = params["durations"].as<std::vector<double>>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.ok;

    } else if (service_name == "/write_string_srv") {
        ros::ServiceClient client = nh_->serviceClient<hand_planner::WriteString>(service_name);
        hand_planner::WriteString srv;
        srv.request.data = params["data"].as<std::string>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.success;

    } else if (service_name == "/joint_command") {
        ros::ServiceClient client = nh_->serviceClient<gait_planner::command>(service_name);
        gait_planner::command srv;
        srv.request.motor_id = params["motor_id"].as<int>();
        srv.request.angle = params["angle"].as<double>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return srv.response.result;

    } else if (service_name == "/get_data") {
        ros::ServiceClient client = nh_->serviceClient<gait_planner::getdata>(service_name);
        gait_planner::getdata srv;
        srv.request.time = params["time"].as<double>();
        if (!client.call(srv)) { ROS_ERROR("Service call to %s failed.", service_name.c_str()); return false; }
        return true; // Assuming this service always succeeds if called
    
    } else {
        ROS_ERROR("Scenario step contains unknown service: %s", service_name.c_str());
        return false;
    }
}

void RobotManager::publishTriggerCallback(const std_msgs::Empty::ConstPtr& msg) {
    ROS_DEBUG("Received publish trigger. Publishing combined commands.");
    publishCombinedMotorCommands(); 
}

void RobotManager::publishCombinedMotorCommands() {
    // Get commands from GaitManager (lower body)
    const double* gait_motor_commands = gait_manager_->getGaitMotorCommands();
    const double* gait_gazebo_commands = gait_manager_->getGaitGazeboCommands();

    // Get commands from HandManager (upper body and head)
    const std::vector<double>& hand_motor_commands = hand_manager_->getHandMotorCommands();
    const std::vector<double>& hand_gazebo_commands = hand_manager_->getHandGazeboCommands();

    // Get commands from HeadManager (upper body and head)
    const std::vector<double>& head_motor_commands = hand_manager_->getHeadMotorCommands();
    const std::vector<double>& head_gazebo_commands = hand_manager_->getHeadGazeboCommands();

    // Get commands from FingerControl (hands)
    const std::vector<double>& finger_motor_commands = hand_manager_->getFingerMotorCommands();    

    if (!simulation)
    {
        combined_motor_command_msg_.data.clear();
        combined_motor_command_msg_.data.reserve(12 + 8 + 3 + 6 + finger_motor_commands.size());

        // -------------------------
        // 0–11 : Gait
        // -------------------------
        for (int i = 0; i < 12; i++)
            combined_motor_command_msg_.data.push_back(gait_motor_commands[i]);

        // -------------------------
        // 12–19 : Hand motor (first part)
        // -------------------------
        for (int i = 12; i < 20; i++)
            combined_motor_command_msg_.data.push_back(hand_motor_commands[i]);

        // -------------------------
        // 20–22 : Head motor (NEW)
        // -------------------------
        combined_motor_command_msg_.data.push_back(head_motor_commands[0]);
        combined_motor_command_msg_.data.push_back(head_motor_commands[1]);
        combined_motor_command_msg_.data.push_back(head_motor_commands[2]);

        // -------------------------
        // 23–29 : Hand motor (remaining)
        // -------------------------
        for (int i = 23; i < 29; i++)
            combined_motor_command_msg_.data.push_back(hand_motor_commands[i]);

        // -------------------------
        // Finger motors
        // -------------------------
        for (int i = 0; i < 17; ++i) {
            combined_motor_command_msg_.data.push_back(finger_motor_commands[i]);
        }
        // -------------------------
        // Override shoulders if gait contributes
        // -------------------------
        if (gait_motor_commands[12] != 0)  
            combined_motor_command_msg_.data[12] += gait_motor_commands[12];

        if (gait_motor_commands[16] != 0)
            combined_motor_command_msg_.data[16] += gait_motor_commands[16];

        combined_motor_pub_.publish(combined_motor_command_msg_);


        // for (size_t i = 0; i < combined_motor_command_msg_.data.size(); ++i) {
        //     std::cout << combined_motor_command_msg_.data[i];
        //     if (i < combined_motor_command_msg_.data.size() - 1) std::cout << ", ";
        // }
        // std::cout << std::endl;
    }
    else
    {
        combined_gazebo_command_msg_.data.clear();
        combined_gazebo_command_msg_.data.reserve(12 + 8 + 3 + 6 + finger_motor_commands.size());

        // -------------------------
        // 0–11 : Gait (rad)
        // -------------------------
        for (int i = 0; i < 12; i++)
            combined_gazebo_command_msg_.data.push_back(gait_gazebo_commands[i]);

        // -------------------------
        // 12–19 : Hand gazebo (first part)
        // -------------------------
        for (int i = 12; i < 20; i++)
            combined_gazebo_command_msg_.data.push_back(hand_gazebo_commands[i]);

        // -------------------------
        // 20–22 : Head gazebo (rad)
        // -------------------------
        combined_gazebo_command_msg_.data.push_back(head_gazebo_commands[0]);
        combined_gazebo_command_msg_.data.push_back(head_gazebo_commands[1]);
        combined_gazebo_command_msg_.data.push_back(head_gazebo_commands[2]);

        // -------------------------
        // 23–29 : Hand gazebo (remaining)
        // -------------------------
        for (int i = 23; i < 29; i++)
            combined_gazebo_command_msg_.data.push_back(hand_gazebo_commands[i]);

        // -------------------------
        // Finger commands
        // -------------------------
        for (int i = 0; i < 17; ++i) {
            combined_gazebo_command_msg_.data.push_back(finger_motor_commands[i]);
        }
        // -------------------------
        // Override shoulders if gait provides values
        // -------------------------
        if (gait_gazebo_commands[12] != 0.0)
            combined_gazebo_command_msg_.data[12] = gait_gazebo_commands[12];

        if (gait_gazebo_commands[16] != 0.0)
            combined_gazebo_command_msg_.data[16] = gait_gazebo_commands[16];

        combined_gazebo_pub_.publish(combined_gazebo_command_msg_);

        // for (size_t i = 0; i < combined_gazebo_command_msg_.data.size(); ++i) {
        //     std::cout << combined_gazebo_command_msg_.data[i];
        //     if (i < combined_gazebo_command_msg_.data.size() - 1) std::cout << ", ";
        // }
        // std::cout << std::endl;
    }

}


// --- Main Function ---
int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_manager_node");
    ros::NodeHandle n;
    RobotManager robot_manager(&n);
    ROS_INFO("Robot Manager is running and ready to execute commands and scenarios.");
    // Create an AsyncSpinner.
    // The '2' means it will create a pool of 2 threads to process callbacks and prevent deadlocks (one for the scenario client, one for the service provider). You can increase this if needed.
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown(); // replacement of the old ros::spin().
    return 0;
}