#pragma once
#ifndef HAND_MANAGER_H
#define HAND_MANAGER_H

#include "S5_hand.h"
#include "MinimumJerkInterpolation.h"
#include "hand_motion_utils.h"
#include "handwriting.h"
#include "FingerControl.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include "json.hpp"
#include <deque>
#include <mutex>

// msgs & srvs
#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include "hand_planner/DetectionInfoArray.h"
#include "hand_planner/move_hand_single.h"
#include "hand_planner/move_hand_both.h"
#include "hand_planner/gripOnline.h"
#include "hand_planner/SetTargetClass.h"
#include "hand_planner/head_track.h"
#include "hand_planner/WriteString.h"
#include "hand_planner/PickAndMove.h"
#include "hand_planner/KeyboardJog.h"
#include "hand_planner/MoveHandGeneral.h"
#include "hand_planner/FingerControl.h"
#include "hand_planner/FingerScenario.h"
#include "hand_planner/arm_back_to_home.h"

using namespace std;
using namespace Eigen;
using json = nlohmann::json;

class HandManager {
public:
    HandManager(ros::NodeHandle *n);

private:
    ros::Publisher trajectory_data_pub;
    ros::Publisher gazeboJointStatePub_;
    ros::Subscriber camera_data_sub;
    ros::Subscriber joint_qc_sub;
    ros::Subscriber teleoperation_data_sub;
    ros::Subscriber micArray_data_sub;
    ros::ServiceServer move_hand_single_service;
    ros::ServiceServer move_hand_both_service;
    ros::ServiceServer grip_online_service;
    ros::ServiceServer set_target_class_service;
    ros::ServiceServer head_track_service;
    ros::ServiceServer teleoperation_service;
    ros::ServiceServer write_string_service_;
    ros::ServiceServer move_hand_relative_service_;
    ros::ServiceServer move_hand_keyboard_service_;
    ros::ServiceServer move_hand_general_service_;
    ros::ServiceServer finger_control_service_;
    ros::ServiceServer finger_scenario_service_;
    ros::ServiceServer arm_back_to_home_service_;
    ros::Subscriber hand_keyboard_sub_;
  
    ros::WallTime hand_keyboard_last_input_;
    double hand_step_T_ = 1.0;
    double hand_max_dx_ = 0.08; 
    double hand_max_dyz_ = 0.30;

    S5_hand hand_func_R;
    S5_hand hand_func_L;
    MinimumJerkInterpolation coef_generator;
    
    // Finger control
    std::unique_ptr<FingerControl> finger_control_;

    VectorXd q_ra;
    VectorXd q_la;
    VectorXd q_init_r;
    VectorXd q_init_l;
    MatrixXd qref_r;
    MatrixXd qref_l;
    VectorXd next_ini_ee_posR;
    VectorXd next_ini_ee_posL;
    vector<double> last_q_gazebo;
    vector<double> last_q_motor;

    
    double sum_r;
    double sum_l;
    int QcArr[29];
    std_msgs::Float64MultiArray joint_angles_gazebo_;
    VectorXd q_rad_teleop;

    Eigen::VectorXd q_right_state_;
    Eigen::VectorXd q_right_baseline_;
    Eigen::Matrix3d R_right_state_;

    Eigen::VectorXd q_left_state_;
    Eigen::VectorXd q_left_baseline_;
    Eigen::Matrix3d R_left_state_;


    double T;
    int rate;
    bool simulation;
    bool right_state_init_;
    bool left_state_init_;

    bool hand_keyboard_enabled_ =false;
    int encoderResolution[2];
    int harmonicRatio[4];
    vector<int> pitch_range, roll_range, yaw_range;
    vector<int> pitch_command_range, roll_command_range, yaw_command_range;
    vector<int> wrist_command_range, wrist_yaw_range, wrist_right_range, wrist_left_range;

    double X, Y, Z;
    double tempX, tempY, tempZ;
    double h_pitch, h_roll, h_yaw;
    double Kp, Ky;
    double t_grip;
    double micArray_theta;
    int target_class_id_ = 41;
    std::mutex target_mutex_;
    std::deque<double> micArray_data_buffer;

    void object_detect_callback(const hand_planner::DetectionInfoArray &msg);
    void joint_qc_callback(const std_msgs::Int32MultiArray::ConstPtr &qcArray);
    void teleoperation_callback(const std_msgs::Float64MultiArray &q_deg_teleop);
    void micArray_callback(const std_msgs::Float64 &msg);
    void hand_keyboard_callback(const std_msgs::Int32::ConstPtr& msg);
    void publishMotorData(const VectorXd& q_rad_right, const VectorXd& q_rad_left, const Vector3d& head_angles);

    MatrixXd scenario_target(HandType type, string scenario, int i, VectorXd ee_pos, string ee_ini_pos);
    VectorXd reach_target(S5_hand& hand_model, VectorXd& q_arm, MatrixXd& qref_arm, double& sum_arm, VectorXd& q_init_arm, MatrixXd targets, string scenario, int M);
    Vector2d head_follow_hand(HandType type, const VectorXd& q_arm);

    bool single_hand(hand_planner::move_hand_single::Request &req, hand_planner::move_hand_single::Response &res);
    bool both_hands(hand_planner::move_hand_both::Request &req, hand_planner::move_hand_both::Response &res);
    bool grip_online(hand_planner::gripOnline::Request &req, hand_planner::gripOnline::Response &res);
    bool setTargetClassService(hand_planner::SetTargetClass::Request &req, hand_planner::SetTargetClass::Response &res);
    bool head_track_handler(hand_planner::head_track::Request &req, hand_planner::head_track::Response &res);
    bool teleoperation_handler(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool write_string_handler(hand_planner::WriteString::Request &req, hand_planner::WriteString::Response &res);
    bool move_hand_relative_handler(hand_planner::PickAndMove::Request &req, hand_planner::PickAndMove::Response &res);
    bool move_hand_keyboard_handler(hand_planner::KeyboardJog::Request &req, hand_planner::KeyboardJog::Response &res);
    bool move_hand_general_handler(hand_planner::MoveHandGeneral::Request &req, hand_planner::MoveHandGeneral::Response &res);
    
    // Finger control services
    bool fingerControlService(hand_planner::FingerControl::Request &req, hand_planner::FingerControl::Response &res);
    bool fingerScenarioService(hand_planner::FingerScenario::Request &req, hand_planner::FingerScenario::Response &res);
    // Finger control access methods
    FingerControl* getFingerControl() { return finger_control_.get(); }
  
    bool arm_back_to_home_handler(hand_planner::arm_back_to_home::Request &req, hand_planner::arm_back_to_home::Response &res);
};

#endif
