#include "HandManager.h"

// --- CONSTRUCTOR ---
HandManager::HandManager(ros::NodeHandle *n) : 
    hand_func_R(RIGHT), 
    hand_func_L(LEFT),
    // Initialize parameters
    T(0.005),
    rate(200),
    simulation(false),
    X(1.0), Y(0.0), Z(0.0),
    tempX(1.0), tempY(0.0), tempZ(0.0),
    h_pitch(0), h_roll(0), h_yaw(0),
    Kp(0.01), Ky(-0.01),
    t_grip(0),
    sum_r(0), sum_l(0)
{
    // Motor and sensor constants
    encoderResolution[0] = 4096 * 4;
    encoderResolution[1] = 2048 * 4;
    harmonicRatio[0] = 100;
    harmonicRatio[1] = 100;
    harmonicRatio[2] = 100;
    harmonicRatio[3] = 400;
    
    // Parameter vectors
    pitch_range = {-30, 30};
    roll_range = {-50, 50};
    yaw_range = {-90, 90};
    pitch_command_range = {180, 140};
    roll_command_range = {100, 200};
    yaw_command_range = {90, 210};
    wrist_command_range = {0, 180};
    wrist_yaw_range = {90, -90};
    wrist_right_range = {90, -90};
    wrist_left_range = {90, -90};

    q_rad_teleop.resize(14);
    q_rad_teleop.setZero();

    last_q_gazebo.resize(29, 0.0);
    last_q_motor.resize(29, 0.0);

    // initial position (right hand)
    q_right_state_.resize(7);
    q_right_state_ << 10*M_PI/180.0, -10*M_PI/180.0, 0, -25*M_PI/180.0, 0, 0, 0;
    R_right_state_.setIdentity();
    right_state_init_ = true;
    
    // Initialize finger control
    finger_control_ = std::make_unique<FingerControl>(n);
    
    // initial position (left hand)
    q_left_state_.resize(7);
    q_left_state_ << 10*M_PI/180.0, 10*M_PI/180.0, 0, -25*M_PI/180.0, 0, 0, 0;
    R_left_state_.setIdentity();
    left_state_init_ = true;

    // ROS Communication Setup
    trajectory_data_pub = n->advertise<std_msgs::Int32MultiArray>("jointdata/qc", 100);
    gazeboJointStatePub_ = n->advertise<std_msgs::Float64MultiArray>("/joint_angles_gazebo", 100);
    camera_data_sub = n->subscribe("/detection_info", 1, &HandManager::object_detect_callback, this);
    joint_qc_sub = n->subscribe("jointdata/qc", 100, &HandManager::joint_qc_callback, this);
    teleoperation_data_sub = n->subscribe("teleoperation/angles", 100, &HandManager::teleoperation_callback, this);
    micArray_data_sub = n->subscribe("micarray/angle", 100, &HandManager::micArray_callback, this);
    move_hand_single_service = n->advertiseService("move_hand_single_srv", &HandManager::single_hand, this);
    move_hand_both_service = n->advertiseService("move_hand_both_srv", &HandManager::both_hands, this);
    grip_online_service = n->advertiseService("grip_online_srv", &HandManager::grip_online, this);
    set_target_class_service = n->advertiseService("set_target_class_srv", &HandManager::setTargetClassService, this);
    head_track_service = n->advertiseService("head_track_srv", &HandManager::head_track_handler, this);
    teleoperation_service = n->advertiseService("teleoperation_srv", &HandManager::teleoperation_handler, this);
    write_string_service_        = n->advertiseService("write_string_srv",        &HandManager::write_string_handler, this);
    move_hand_relative_service_  = n->advertiseService("move_hand_relative_srv",  &HandManager::move_hand_relative_handler, this);
    move_hand_keyboard_service_  = n->advertiseService("move_hand_keyboard_srv",  &HandManager::move_hand_keyboard_handler, this);
    move_hand_general_service_   = n->advertiseService("move_hand_general_srv",   &HandManager::move_hand_general_handler, this);
    hand_keyboard_sub_           = n->subscribe("/keyboard_command", 10, &HandManager::hand_keyboard_callback, this);
    arm_back_to_home_service_    = n->advertiseService("arm_back_to_home_srv",    &HandManager::arm_back_to_home_handler, this);
    finger_control_service_  = n->advertiseService("finger_control_srv", &HandManager::fingerControlService, this);
    finger_scenario_service_ = n->advertiseService("finger_scenario_srv", &HandManager::fingerScenarioService, this);
}

// --- Object Detection Callback Implementations ---
bool HandManager::setTargetClassService(hand_planner::SetTargetClass::Request &req, hand_planner::SetTargetClass::Response &res) {
        // Look up class ID from JSON
        string object_classes_path = ros::package::getPath("hand_planner") + "/config/object_classes.json";
        std::ifstream fr(object_classes_path);
        json object_classes = json::parse(fr);
        if (!object_classes.contains(req.class_name)) {
            ROS_ERROR("Class name '%s' not found in object_classes.json!", req.class_name.c_str());
            res.class_id = -1;
        } else {
            target_class_id_ = object_classes[req.class_name];    
            res.class_id = target_class_id_;
            X = 1.0; Y = 0; Z = 0; 
        }
        return true;
}

void HandManager::object_detect_callback(const hand_planner::DetectionInfoArray &msg) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (size_t i = 0; i < msg.detections.size(); ++i) {
        if (msg.detections[i].class_id == target_class_id_) {
            double dist = msg.detections[i].distance / 1000.0;
            double y_pixel = msg.detections[i].x + msg.detections[i].width / 2.0;
            double z_pixel = msg.detections[i].y + msg.detections[i].height / 2.0;
            double a = 0.5, b = 0.4, X0 = 0.5;
            int L = 640, W = 480;

            double Y0 = -(y_pixel - L / 2.0) / L * a;
            double Z0 = -(z_pixel - W / 2.0) / W * b;
            double L0 = sqrt(pow(X0, 2) + pow(Y0, 2) + pow(Z0, 2));

            X = X0 * dist / L0;
            Y = Y0 * dist / L0;
            Z = Z0 * dist / L0;
            tempX = X; tempY = Y; tempZ = Z;
            return;
        }
    }
    X = tempX; Y = tempY; Z = tempZ;
}

void HandManager::joint_qc_callback(const std_msgs::Int32MultiArray::ConstPtr &qcArray) {
    for (size_t i = 0; i < qcArray->data.size() && i < 29; ++i) {
        QcArr[i] = qcArray->data[i];
        // cout << QcArr[i] << " ";
    }
    // cout << endl;
}

void HandManager::teleoperation_callback(const std_msgs::Float64MultiArray &q_deg_teleop) {
    assert(q_deg_teleop.data.size() == 14);
    for (size_t i = 0; i < q_deg_teleop.data.size(); ++i) {
        q_rad_teleop(i) = q_deg_teleop.data[i] * M_PI / 180.0;  // deg → rad
    }
}

void HandManager::micArray_callback(const std_msgs::Float64 &msg) {
    int MAX_CAPACITY = 10;
    micArray_data_buffer.push_back(msg.data);
    if (micArray_data_buffer.size() > MAX_CAPACITY) {
        micArray_data_buffer.pop_front();
    }
    // Only check if we have at least 5 values
    if (micArray_data_buffer.size() >= 3) {
        auto start = micArray_data_buffer.end() - 3;    // Take last 3 elements
        bool all_equal = std::all_of(start + 1, micArray_data_buffer.end(), [first = *start](double v){ return fabs(v - first) < 1e-6; });
        if (all_equal) {
            micArray_theta = (msg.data - 90) * M_PI / 180;
        }
    }
}

MatrixXd HandManager::scenario_target(HandType type, string scenario, int i, VectorXd ee_pos, string ee_ini_pos) {
    MatrixXd result(6, 3);
    VectorXd r_middle, r_target, r_start;
    MatrixXd R_target;

    S5_hand& hand_func = (type == RIGHT) ? hand_func_R : hand_func_L;
    VectorXd& q_arm = (type == RIGHT) ? q_ra : q_la;
    VectorXd& q_init = (type == RIGHT) ? q_init_r : q_init_l;
    VectorXd& next_ee_pos = (type == RIGHT) ? next_ini_ee_posR : next_ini_ee_posL;

    if (scenario == "shakeHands") {
        r_middle = (type == RIGHT) ? Vector3d(0.35, -0.1, -0.2) : Vector3d(0.35, 0.1, -0.2);
        r_target = (type == RIGHT) ? Vector3d(0.3, -0.03, -0.3) : Vector3d(0.3, 0.03, -0.3);
        R_target = hand_func.rot(2, -65 * M_PI / 180, 3);
    } else if (scenario == "respect") {
        r_middle = (type == RIGHT) ? Vector3d(0.3, -0.1, -0.3) : Vector3d(0.3, 0.1, -0.3);
        r_target = (type == RIGHT) ? Vector3d(0.25, 0.2, -0.3) : Vector3d(0.25, -0.2, -0.3);
        double rot_angle = (type == RIGHT) ? 60 * M_PI / 180 : -60 * M_PI / 180;
        R_target = hand_func.rot(2, -80 * M_PI / 180, 3) * hand_func.rot(1, rot_angle, 3);
    } else if (scenario == "byebye") {
        r_middle = (type == RIGHT) ? Vector3d(0.35, -0.1, -0.15) : Vector3d(0.35, 0.2, -0.15);
        r_target = (type == RIGHT) ? Vector3d(0.3, 0, 0.22) : Vector3d(0.3, 0.1, 0.22);
        R_target = hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot(1, -160 * M_PI / 180, 3);
    } else if (scenario == "punching") {
        r_middle = (type == RIGHT) ? Vector3d(0.15, -0.1, -0.1) : Vector3d(0.15, 0.1, -0.1);
        r_target = (type == RIGHT) ? Vector3d(0.35, 0.2, 0.1) : Vector3d(0.35, -0.2, 0.1);
        R_target = hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot(1, -40 * M_PI / 180, 3);
    } else if (scenario == "perfect") {
        r_middle = (type == RIGHT) ? Vector3d(0.15, -0.1, -0.3) : Vector3d(0.15, 0.1, -0.3);
        r_target = (type == RIGHT) ? Vector3d(0.25, -0.05, -0.25) : Vector3d(0.25, 0.05, -0.25);
        R_target = hand_func.rot(2, -90 * M_PI / 180, 3) * hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot(1, -45 * M_PI / 180, 3);
    } else if (scenario == "pointing") {
        r_middle = (type == RIGHT) ? Vector3d(0.25, -0.1, -0.1) : Vector3d(0.25, 0.1, -0.1);
        r_target = (type == RIGHT) ? Vector3d(0.45, 0.05, 0.0) : Vector3d(0.45, -0.05, 0.0);
        R_target = hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot(1, -65 * M_PI / 180, 3);
    } else if (scenario == "like") {
        r_middle = (type == RIGHT) ? Vector3d(0.15, -0.1, -0.3) : Vector3d(0.15, 0.1, -0.3);
        r_target = (type == RIGHT) ? Vector3d(0.25, -0.05, -0.25) : Vector3d(0.25, 0.05, -0.25);
        R_target = hand_func.rot(2, -90 * M_PI / 180, 3);
    } else if (scenario == "home") {
        r_middle = (type == RIGHT) ? Vector3d(0.3, -0.1, -0.25) : Vector3d(0.3, 0.1, -0.25);
        r_target = (type == RIGHT) ? Vector3d(0.15, -0.07, -0.43) : Vector3d(0.15, 0.07, -0.43);
        R_target = hand_func.rot(2, -20 * M_PI / 180, 3);
    }

    q_arm.resize(7);
    q_init.resize(7);
    if (i == 0) {
        if (ee_ini_pos == "init") {
            if (type == RIGHT) {
                q_arm << 10, -10, 0, -25, 0, 0, 0;
            } else { // LEFT
                q_arm << 10, 10, 0, -25, 0, 0, 0;
            }
            q_arm *= M_PI / 180.0;
            q_init = q_arm;
            hand_func.HO_FK_palm(q_arm);
            r_start = hand_func.r_palm;
        } else { 
            r_start = next_ee_pos; }
    } else { 
        r_start = ee_pos; }

    result << r_middle.transpose(), r_target.transpose(), R_target.row(0), R_target.row(1), R_target.row(2), r_start.transpose();
    return result;
}

VectorXd HandManager::reach_target(S5_hand& hand_model, VectorXd& q_arm, MatrixXd& qref_arm, double& sum_arm, VectorXd& q_init_arm, MatrixXd targets, string scenario, int M) {
    qref_arm.resize(7, M);
    Vector3d r_middle = targets.row(0), r_target = targets.row(1), r_start = targets.row(5);
    Matrix3d R_target = targets.block(2, 0, 3, 3);
    
    MatrixXd t_r(1, 3); 
    t_r << 0, 2, 4;
    double total_time = (scenario == "byebye" || scenario == "shakeHands") ? t_r(2) + 4 : t_r(2);
    int step_count = total_time / T;
    
    MatrixXd P_x(1, 3), P_y(1, 3), P_z(1, 3);
    P_x << r_start(0), r_middle(0), r_target(0);
    P_y << r_start(1), r_middle(1), r_target(1);
    P_z << r_start(2), r_middle(2), r_target(2);
    MatrixXd V_inf(1,3); V_inf << 0, INFINITY, 0; 
    MatrixXd A_inf(1,3); A_inf << 0, INFINITY, 0;

    // define minJerk elements to calculate end effector velocity
    MatrixXd X_coef = coef_generator.Coefficient(t_r, P_x, V_inf, A_inf);
    MatrixXd Y_coef = coef_generator.Coefficient(t_r, P_y, V_inf, A_inf);
    MatrixXd Z_coef = coef_generator.Coefficient(t_r, P_z, V_inf, A_inf);

    for (int count = 0; count < step_count; ++count) {
        double time_r = count * T;
        Vector3d V_curr;
        if (time_r < t_r(2)) {
            int interval = (time_r < t_r(1)) ? 0 : 1;
            double t_interval = (time_r < t_r(1)) ? t_r(0) : t_r(1);
            V_curr << coef_generator.GetAccVelPos(X_coef.row(interval), time_r, t_interval, 5)(0,1),
                      coef_generator.GetAccVelPos(Y_coef.row(interval), time_r, t_interval, 5)(0,1),
                      coef_generator.GetAccVelPos(Z_coef.row(interval), time_r, t_interval, 5)(0,1);
            hand_model.update_hand(q_arm, V_curr, r_target, R_target);
        } else {
            if (scenario == "byebye") { 
                q_arm(2) += (hand_model.hand_type == RIGHT ? -0.5 : 0.5) * M_PI/180 * cos((time_r - t_r(2)) * (2 * M_PI)); }
            else if (scenario == "shakeHands") { 
                q_arm(3) -= 0.125 * M_PI/180 * cos((time_r - t_r(2)) * M_PI); }
            hand_model.update_hand(q_arm, Vector3d::Zero(), r_target, R_target);
        }
        hand_model.doQP(q_arm);
        q_arm = hand_model.q_next;
        int global_index = count + sum_arm / T;
        if (global_index < qref_arm.cols()) { 
            qref_arm.col(global_index) = q_arm - q_init_arm; }
    }
    sum_arm += total_time;
    hand_model.HO_FK_palm(q_arm);
    return hand_model.r_palm;
}

Vector2d HandManager::head_follow_hand(HandType type, const VectorXd& q_arm)
{
    double dx = 0.00;
    double dy = (type==RIGHT)? -0.2 : 0.2;
    double dz = -0.25;

    static const double YAW_MIN=-60.0*M_PI/180.0, YAW_MAX=60.0*M_PI/180.0;
    static const double PITCH_MIN=-28.0*M_PI/180.0, PITCH_MAX=28.0*M_PI/180.0;

    auto wrap=[](double a){ while(a> M_PI)a-=2*M_PI; while(a<-M_PI)a+=2*M_PI; return a; };
    auto clamp=[](double v,double lo,double hi){ return std::max(lo,std::min(hi,v)); };

    if (type==RIGHT) hand_func_R.HO_FK_palm(q_arm);
    else             hand_func_L.HO_FK_palm(q_arm);

    const Eigen::Vector3d p_sh = (type==RIGHT)? hand_func_R.r_palm : hand_func_L.r_palm;
    Eigen::Vector3d p_neck = p_sh + Eigen::Vector3d(dx,dy,dz);

    double yaw_des   = std::atan2(p_neck.y(),  p_neck.x());
    double pitch_des = std::atan2(-p_neck.z(), std::hypot(p_neck.y(), p_neck.x()));

    double e_yaw   = wrap(yaw_des   - h_yaw);
    double e_pitch = wrap(pitch_des - h_pitch);

    h_yaw   += -Ky*e_yaw;
    h_pitch += Kp*e_pitch;

    h_yaw   = clamp(h_yaw,   YAW_MIN,   YAW_MAX);
    h_pitch = clamp(h_pitch, PITCH_MIN, PITCH_MAX);

    return Vector2d(h_yaw, h_pitch);
}

void HandManager::publishMotorData(const VectorXd& q_rad_right, const VectorXd& q_rad_left, const Vector3d& head_angles) {
     /* The function automatically:
      * - Converts radians to encoder counts for real robot
      * - Handles wrist calculations
      * - Publishes to appropriate topics (/jointdata/qc or /joint_angles_gazebo)
      * - Supports both simulation and real robot modes
      * - ALWAYS publishes data for all 29 joints (use zero vectors for joints you don't want to control)
    */
    vector<double> q_motor(29, 0.0);
    vector<double> q_gazebo(29, 0.0);

    if (!simulation) {
        // Right hand motors (indices 12-15)
        q_motor[12] = int(q_rad_right(0) * encoderResolution[0] * harmonicRatio[0] / (2 * M_PI));
        q_motor[13] = -int(q_rad_right(1) * encoderResolution[0] * harmonicRatio[1] / (2 * M_PI));
        q_motor[14] = int(q_rad_right(2) * encoderResolution[1] * harmonicRatio[2] / (2 * M_PI));
        q_motor[15] = -int(q_rad_right(3) * encoderResolution[1] * harmonicRatio[3] / (2 * M_PI));
        
        // Left hand motors (indices 16-19)
        q_motor[16] = -int(q_rad_left(0) * encoderResolution[0] * harmonicRatio[0] / (2 * M_PI));
        q_motor[17] = -int(q_rad_left(1) * encoderResolution[0] * harmonicRatio[1] / (2 * M_PI));
        q_motor[18] = int(q_rad_left(2) * encoderResolution[1] * harmonicRatio[2] / (2 * M_PI));
        q_motor[19] = int(q_rad_left(3) * encoderResolution[1] * harmonicRatio[3] / (2 * M_PI));
        
        // Head motors (indices 20-22) - roll, pitch, yaw
        q_motor[20] = int(roll_command_range[0] + (roll_command_range[1] - roll_command_range[0]) * 
                         ((-(head_angles(0)*180/M_PI) - roll_range[0]) / (roll_range[1] - roll_range[0])));
        q_motor[21] = int(pitch_command_range[0] + (pitch_command_range[1] - pitch_command_range[0]) * 
                         ((-(head_angles(1)*180/M_PI) - pitch_range[0]) / (pitch_range[1] - pitch_range[0])));
        q_motor[22] = int(yaw_command_range[0] + (yaw_command_range[1] - yaw_command_range[0]) * 
                         ((-(head_angles(2)*180/M_PI) - yaw_range[0]) / (yaw_range[1] - yaw_range[0])));
        
        // Wrist calculations for right hand (indices 23-25)
        q_motor[23] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * 
                            (((q_rad_right(4) * 180 / M_PI) - wrist_yaw_range[0]) / (wrist_yaw_range[1] - wrist_yaw_range[0])));
        q_motor[24] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * 
                            (((hand_func_R.wrist_right_calc(q_rad_right(5), q_rad_right(6))) - (wrist_right_range[0])) / (wrist_right_range[1] - (wrist_right_range[0]))));
        q_motor[25] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * 
                            (((hand_func_R.wrist_left_calc(q_rad_right(5), q_rad_right(6))) - wrist_left_range[0]) / (wrist_left_range[1] - wrist_left_range[0])));
    
        // Wrist calculations for left hand (indices 26-28)
        q_motor[26] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * 
                            (((q_rad_left(4) * 180 / M_PI) - wrist_yaw_range[0]) / (wrist_yaw_range[1] - wrist_yaw_range[0])));
        q_motor[27] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * 
                            (((hand_func_L.wrist_right_calc(q_rad_left(5), q_rad_left(6))) - (wrist_right_range[0])) / (wrist_right_range[1] - (wrist_right_range[0]))));
        q_motor[28] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * 
                            (((hand_func_L.wrist_left_calc(q_rad_left(5), q_rad_left(6))) - wrist_left_range[0]) / (wrist_left_range[1] - wrist_left_range[0])));
        
        // Publish motor data
        std_msgs::Int32MultiArray trajectory_data;
        for(int i = 0; i < 29; i++) { 
            trajectory_data.data.push_back(q_motor[i]); 
        }
        trajectory_data_pub.publish(trajectory_data);
        // cout << "robot_right" << ", " << q_motor[12] << ", " << q_motor[13] << ", " << q_motor[14] << ", " << q_motor[15] << ", " << q_motor[23] << ", " << q_motor[24] << ", " << q_motor[25] << endl;
        // cout << "robot_left" << ", " << q_motor[16] << ", " << q_motor[17] << ", " << q_motor[18] << ", " << q_motor[19] << ", " << q_motor[26] << ", " << q_motor[27] << ", " << q_motor[28] << endl;
        last_q_motor = q_motor;
        
    } else { // simulation
        // Right hand joints
        q_gazebo[12] = q_rad_right(0);  
        q_gazebo[13] = q_rad_right(1);   
        q_gazebo[14] = q_rad_right(2);  
        q_gazebo[15] = q_rad_right(3);
        
        // Left hand joints
        q_gazebo[16] = q_rad_left(0);  
        q_gazebo[17] = q_rad_left(1);   
        q_gazebo[18] = q_rad_left(2);  
        q_gazebo[19] = q_rad_left(3);
        
        // Head joints
        q_gazebo[20] = -head_angles(0); // roll
        q_gazebo[21] = -head_angles(1); // pitch
        q_gazebo[22] = -head_angles(2); // yaw
        
        // Wrist joints for right hand
        q_gazebo[23] = q_rad_right(4);   
        q_gazebo[24] = q_rad_right(5);  
        q_gazebo[25] = q_rad_right(6);
        
        // Wrist joints for left hand
        q_gazebo[26] = q_rad_left(4);   
        q_gazebo[27] = q_rad_left(5);  
        q_gazebo[28] = q_rad_left(6); 
        
        // Publish gazebo data
        joint_angles_gazebo_.data.clear();
        for (int i = 0; i < 29; i++) {
            joint_angles_gazebo_.data.push_back(q_gazebo[i]);
        }
        gazeboJointStatePub_.publish(joint_angles_gazebo_);
        // cout << "gazebo_sim" << ", " << q_gazebo[12] << ", " << q_gazebo[13] << ", " << q_gazebo[14] << ", " << q_gazebo[15] << ", " << q_gazebo[23] << ", " << q_gazebo[24] << ", " << q_gazebo[25] << endl;
    }
}

// --- Service Handler Implementations ---
bool HandManager::single_hand(hand_planner::move_hand_single::Request &req,
                              hand_planner::move_hand_single::Response &res)
{
    ros::Rate rate_(rate);
    int M = std::max(1, (int)std::floor(req.t_total / T));
    Eigen::VectorXd ee_pos = Eigen::Vector3d::Zero();
    HandType type;
    if (req.mode == "righthand"){
        type = RIGHT;
    }
    else if (req.mode == "lefthand"){
        type = LEFT;
    }
    else {
        ROS_ERROR("Invalid mode: %s (must be 'righthand' or 'lefthand')", req.mode.c_str());
        return false;
    }

    if (type == RIGHT) {
        if (q_right_state_.size() == 7) q_ra = q_right_state_;
        else {
            q_ra.resize(7);
            q_ra << 10.0*M_PI/180.0, -10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
        }
        q_init_r = q_ra;

        if (q_right_baseline_.size() != 7) q_right_baseline_ = q_init_r;

        hand_func_R.HO_FK_palm(q_ra);
        next_ini_ee_posR = hand_func_R.r_palm;

        for (int i = 0; i < req.scen_count; i++) {
            Eigen::MatrixXd result = scenario_target(RIGHT, req.scenario[i], i, ee_pos, "prev");
            ee_pos = reach_target(hand_func_R, q_ra, qref_r, sum_r, q_init_r, result, req.scenario[i], M);
        }

        for (int id = 0; id < qref_r.cols(); ++id) {
            Eigen::VectorXd q_abs  = q_init_r + qref_r.col(id);
            Eigen::VectorXd q_send = q_abs;
            q_send.head(4) = q_abs.head(4) - q_right_baseline_.head(4);
            Eigen::VectorXd q_left = Eigen::VectorXd::Zero(7);

            // Eigen::Vector3d head_angles(0,0,0);
            head_follow_hand(RIGHT, q_send);
            Eigen::Vector3d head_angles(h_roll, -h_pitch, -h_yaw);

            publishMotorData(q_send, q_left, head_angles);
            ros::spinOnce();
            rate_.sleep();
        }

        if (qref_r.cols() > 0) q_right_state_ = q_init_r + qref_r.col(qref_r.cols()-1);
        else q_right_state_ = q_ra;

        sum_r = 0;
        next_ini_ee_posR = ee_pos;
        res.ee_fnl_pos = req.scenario[req.scen_count - 1];
        return true;
    } 
    else if (type == LEFT)
    {
        if (q_left_state_.size() == 7) q_la = q_left_state_;
        else {
            q_la.resize(7);
            q_la << 10.0*M_PI/180.0, 10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
        }
        q_init_l = q_la;

        if (q_left_baseline_.size() != 7) q_left_baseline_ = q_init_l;

        hand_func_L.HO_FK_palm(q_la);
        next_ini_ee_posL = hand_func_L.r_palm;

        for (int i = 0; i < req.scen_count; i++) {
            Eigen::MatrixXd result = scenario_target(LEFT, req.scenario[i], i, ee_pos, "prev");
            ee_pos = reach_target(hand_func_L, q_la, qref_l, sum_l, q_init_l, result, req.scenario[i], M);
        }

        for (int id = 0; id < qref_l.cols(); ++id) {
            Eigen::VectorXd q_abs_l  = q_init_l + qref_l.col(id);
            Eigen::VectorXd q_send_l = q_abs_l;
            q_send_l.head(4) = q_abs_l.head(4) - q_left_baseline_.head(4);

            Eigen::VectorXd q_right = Eigen::VectorXd::Zero(7);

            // Eigen::Vector3d head_angles(0,0,0);
            head_follow_hand(LEFT, q_send_l);
            Eigen::Vector3d head_angles(h_roll, -h_pitch, -h_yaw);

            publishMotorData(q_right, q_send_l, head_angles);
            ros::spinOnce();
            rate_.sleep();
        }

        if (qref_l.cols() > 0) q_left_state_ = q_init_l + qref_l.col(qref_l.cols()-1);
        else q_left_state_ = q_la;

        sum_l = 0;
        next_ini_ee_posL = ee_pos;
        res.ee_fnl_pos = req.scenario[req.scen_count - 1];
        return true;
    }
}


bool HandManager::both_hands(hand_planner::move_hand_both::Request &req, hand_planner::move_hand_both::Response &res) {
    ros::Rate rate_(rate);
    int M_req = std::max(1, (int)std::floor(req.t_total / T));
    Eigen::VectorXd ee_pos_r = Eigen::Vector3d::Zero();
    Eigen::VectorXd ee_pos_l = Eigen::Vector3d::Zero();

    // init RIGHT from last absolute state (or defaults)
    if (q_right_state_.size() == 7) q_ra = q_right_state_;
    else {
        q_ra.resize(7);
        q_ra << 10.0*M_PI/180.0, -10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    }
    q_init_r = q_ra;
    if (q_right_baseline_.size() != 7) q_right_baseline_ = q_init_r;
    hand_func_R.HO_FK_palm(q_ra);
    next_ini_ee_posR = hand_func_R.r_palm;

    // init LEFT from last absolute state (or defaults)
    if (q_left_state_.size() == 7) q_la = q_left_state_;
    else {
        q_la.resize(7);
        q_la << 10.0*M_PI/180.0, 10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    }
    q_init_l = q_la;
    if (q_left_baseline_.size() != 7) q_left_baseline_ = q_init_l;
    hand_func_L.HO_FK_palm(q_la);
    next_ini_ee_posL = hand_func_L.r_palm;

    // --- Trajectory Generation Phase ---
    for (int i = 0; i < req.scenR_count; i++) {
        Eigen::MatrixXd result_r = scenario_target(RIGHT, req.scenarioR[i], i, ee_pos_r, "prev");
        ee_pos_r = reach_target(hand_func_R, q_ra, qref_r, sum_r, q_init_r, result_r, req.scenarioR[i], M_req);
    }
    for (int i = 0; i < req.scenL_count; i++) {
        Eigen::MatrixXd result_l = scenario_target(LEFT, req.scenarioL[i], i, ee_pos_l, "prev");
        ee_pos_l = reach_target(hand_func_L, q_la, qref_l, sum_l, q_init_l, result_l, req.scenarioL[i], M_req);
    }

    int Mr = qref_r.cols();
    int Ml = qref_l.cols();
    int M  = std::max(Mr, Ml);

    // --- Execution Phase ---
    for (int id = 0; id < M; ++id) {
        Eigen::VectorXd q_send_r = Eigen::VectorXd::Zero(7);
        Eigen::VectorXd q_send_l = Eigen::VectorXd::Zero(7);

        if (id < Mr) {
            Eigen::VectorXd q_abs_r = q_init_r + qref_r.col(id);
            q_send_r = q_abs_r;
            q_send_r.head(4) = q_abs_r.head(4) - q_right_baseline_.head(4);
        }
        if (id < Ml) {
            Eigen::VectorXd q_abs_l = q_init_l + qref_l.col(id);
            q_send_l = q_abs_l;
            q_send_l.head(4) = q_abs_l.head(4) - q_left_baseline_.head(4);
        }

        Eigen::Vector3d head_angles(0, 0, 0);
        publishMotorData(q_send_r, q_send_l, head_angles);
        ros::spinOnce();
        rate_.sleep();
    }

    // --- Persist last absolute states for next services ---
    if (Mr > 0) q_right_state_ = q_init_r + qref_r.col(Mr-1);
    else        q_right_state_ = q_ra;

    if (Ml > 0) q_left_state_  = q_init_l + qref_l.col(Ml-1);
    else        q_left_state_  = q_la;

    sum_r = 0;
    sum_l = 0;
    next_ini_ee_posR = ee_pos_r;
    next_ini_ee_posL = ee_pos_l;

    res.ee_fnl_posR = req.scenarioR.empty() ? "" : req.scenarioR[req.scenR_count - 1];
    res.ee_fnl_posL = req.scenarioL.empty() ? "" : req.scenarioL[req.scenL_count - 1];
    return true;
}



bool HandManager::grip_online(hand_planner::gripOnline::Request &req, hand_planner::gripOnline::Response &res) {
    ros::Rate rate_(rate);

    Eigen::VectorXd current_q_ra(7);
    Eigen::VectorXd q(7);
    if (q_right_state_.size()==7) q = q_right_state_;
    else q << 10*M_PI/180.0, -10*M_PI/180.0, 0, -25*M_PI/180.0, 0, 0, 0;
    
    current_q_ra = q;
    Eigen::VectorXd initial_q_ra = current_q_ra;
    Eigen::Matrix3d R_target_r = Matrix3d::Identity();

    t_grip = 0;
    while (t_grip <= (120)) {
        Vector3d target2camera(X, Y, Z);
        MatrixXd T_CAM2SH = hand_func_R.ObjToNeck(-h_pitch, h_roll, -h_yaw);
        Vector3d target2shoulder = T_CAM2SH.block(0, 3, 3, 1) + T_CAM2SH.block(0, 0, 3, 3) * target2camera;
        
        // Head Pitch
        if (abs(target2camera(2)) > 0.02) {
            h_pitch += Kp * atan2(target2camera(2), sqrt(pow(target2camera(1),2) + pow(target2camera(0),2)));
            h_pitch = max(-28.0*M_PI/180, min(28.0*M_PI/180, h_pitch));
        }
        // Head Yaw
        if (abs(target2camera(1)) > 0.02) {
            h_yaw += Ky * atan2(target2camera(1), target2camera(0));
            h_yaw = max(-60.0*M_PI/180, min(60.0*M_PI/180, h_yaw));
        }

        if (t_grip >= 15 && Y != 0 && Z != 0) {
        R_target_r = hand_func_R.rot(2, -65 * M_PI / 180, 3);
        hand_func_R.update_hand(current_q_ra, Vector3d::Zero(), target2shoulder, R_target_r);
        Vector3d V_r = 0.7 * (target2shoulder - hand_func_R.r_palm);
        
        hand_func_R.update_hand(current_q_ra, V_r, target2shoulder, R_target_r);
        hand_func_R.doQP(current_q_ra);  // Solve the inverse kinematics
        current_q_ra = hand_func_R.q_next;
        }

        VectorXd q_delta = current_q_ra - initial_q_ra;
        VectorXd q_rad_left = VectorXd::Zero(7);
        Vector3d head_angles(h_roll, h_pitch, h_yaw);
        publishMotorData(q_delta, q_rad_left, head_angles);
        ros::spinOnce();
        rate_.sleep();
        t_grip += T;
    }


    res.finish = "end";
    q_right_state_= current_q_ra;
    return true;
}

bool HandManager::head_track_handler(hand_planner::head_track::Request &req, hand_planner::head_track::Response &res) {
    ROS_INFO("Starting head tracking for %.2f seconds.", req.duration_seconds);
    ros::Rate rate_(rate);
    ros::Time start_time = ros::Time::now();

    while (ros::ok() && (ros::Time::now() - start_time).toSec() < req.duration_seconds) {
        Vector3d target2camera(X, Y, Z);
        
        if (Y != 0 && Z != 0){ // target not in robot's sight; so the head cannot track object yet.
            // Head Pitch
            if (abs(target2camera(2)) > 0.02) {
                h_pitch += Kp * atan2(target2camera(2), sqrt(pow(target2camera(1), 2) + pow(target2camera(0), 2)));
                h_pitch = max(-28.0 * M_PI / 180, min(28.0 * M_PI / 180, h_pitch));
            }
            // Head Yaw
            if (abs(target2camera(1)) > 0.02) {
                h_yaw += Ky * atan2(target2camera(1), target2camera(0));
                h_yaw = max(-60.0 * M_PI / 180, min(60.0 * M_PI / 180, h_yaw));
            }
        } else { // follow direction of arrival (voice) instead.
            h_yaw += Ky * micArray_theta;
            h_yaw = max(-60.0 * M_PI / 180, min(60.0 * M_PI / 180, h_yaw));
        }

        VectorXd q_rad_right = VectorXd::Zero(7);
        VectorXd q_rad_left = VectorXd::Zero(7);
        Vector3d head_angles(h_roll, h_pitch, h_yaw);
        publishMotorData(q_rad_right, q_rad_left, head_angles);
        ros::spinOnce();
        rate_.sleep();
    }
    ROS_INFO("Head tracking finished.");
    res.success = true;
    return true;
}

bool HandManager::teleoperation_handler(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ros::Rate rate_(rate);
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 120) {
        VectorXd q_rad_right = q_rad_teleop.segment(0, 7);
        VectorXd q_rad_left = q_rad_teleop.segment(7, 7);
        Vector3d head_angles(0, 0, 0);
        publishMotorData(q_rad_right, q_rad_left, head_angles);
        ros::spinOnce();
        rate_.sleep();
    }
    return true;
}

bool HandManager::write_string_handler(hand_planner::WriteString::Request &req,
                                       hand_planner::WriteString::Response &res)
{
    VectorXd q(7);
    if (q_right_state_.size()==7) q = q_right_state_;
    else q << 10*M_PI/180.0, -10*M_PI/180.0, 0, -25*M_PI/180.0, 0, 0, 0;

    if (q_right_baseline_.size()!=7) q_right_baseline_ = q;

    Vector3d r_target(0.45, 0.02, 0.03);
    Matrix3d R_target = hand_func_R.rot(2, -140.0*M_PI/180.0, 3)
                      * hand_func_R.rot(1,  -25.0*M_PI/180.0, 3)
                      * hand_func_R.rot(3,   30.0*M_PI/180.0, 3);

    auto pub = [&](const VectorXd& q_abs){
        VectorXd q_send = q_abs;
        q_send.head(4) = q_abs.head(4) - q_right_baseline_.head(4);
        VectorXd ql = VectorXd::Zero(7);
        publishMotorData(q_send, ql, Vector3d(0,0,0));
    };

    if (!approachWhiteboard(hand_func_R, coef_generator, q, r_target, R_target, T, pub)) { res.success=false; return true; }
    writeStringCore(hand_func_R, q, req.data, r_target, R_target, T, pub);

    q_right_state_ = q;
    res.success = true;
    return true;
}


bool HandManager::move_hand_relative_handler(hand_planner::PickAndMove::Request &req,
                                             hand_planner::PickAndMove::Response &res)
{
    VectorXd q(7);
    if (q_right_state_.size()==7) q = q_right_state_;
    else q << 10*M_PI/180.0, -10*M_PI/180.0, 0, -25*M_PI/180.0, 0, 0, 0;

    if (q_right_baseline_.size()!=7) q_right_baseline_ = q;

    Matrix3d R_pick = hand_func_R.rot(2, -100.0*M_PI/180.0, 3);
    Vector3d mid(0.15, -0.15, -0.30);
    Vector3d goal(0.35, -0.10, -0.20);

    auto pub = [&](const VectorXd& q_abs){
        VectorXd q_send = q_abs;
        q_send.head(4) = q_abs.head(4) - q_right_baseline_.head(4);
        VectorXd ql = VectorXd::Zero(7);
        publishMotorData(q_send, ql, Vector3d(0,0,0));
    };

    if (!approachViaOneMid(hand_func_R, coef_generator, q, mid, goal, R_pick, 3.0, 3.0, T, pub)) {
        res.ok = false; res.message = "approach failed"; return true;
    }

    if (req.axes.size() != req.deltas.size() || req.axes.size() != req.durations.size()) {
        res.ok=false; res.message="size mismatch"; return true;
    }

    double MAX_DX = 0.08, MAX_DYZ = 0.30, MIN_DUR = 0.2;
    for (size_t i=0;i<req.axes.size();++i){
        char ax=0; for(char c: req.axes[i]){ if (std::isalpha((unsigned char)c)){ char u=std::toupper((unsigned char)c); if(u=='X'||u=='Y'||u=='Z'){ ax=u; break; } } }
        if (!ax) continue;
        double d=req.deltas[i], dur=std::max(req.durations[i], MIN_DUR);
        Vector3d dxyz(0,0,0);
        if (ax=='X'){ d = std::max(std::min(d, MAX_DX), -MAX_DX); dxyz(0)=d; }
        else if (ax=='Y'){ d = std::max(std::min(d, MAX_DYZ), -MAX_DYZ); dxyz(1)=d; }
        else { d = std::max(std::min(d, MAX_DYZ), -MAX_DYZ); dxyz(2)=d; }

        if (!moveRelative(hand_func_R, coef_generator, q, dxyz, goal, R_pick, dur, T, pub)) {
            res.ok=false; res.message="move failed"; return true;
        }
    }

    q_right_state_ = q;
    res.ok=true; res.message="ok";
    return true;
}


void HandManager::hand_keyboard_callback(const std_msgs::Int32::ConstPtr& msg)
{
    if (!hand_keyboard_enabled_) return;
    const double STEP_T = 1.0;
    const double MAX_DX = 0.08, MAX_DYZ = 0.30;

    int code = msg->data;
    double dx=0, dy=0, dz=0;
    switch (code) {
        case 'w': case 'W': dx = +0.05; break; // Forward
        case 's': case 'S': dx = -0.05; break; // Backward
        case 'e': case 'E': dy = +0.10; break; // To the left
        case 'd': case 'D': dy = -0.10; break; // To the right
        case 'q': case 'Q': dz = +0.10; break; // Upward
        case 'a': case 'A': dz = -0.10; break; // Downward
        default: return;
    }

    if (q_right_state_.size()!=7) return;
    if (dx!=0) dx = std::max(std::min(dx,  MAX_DX),  -MAX_DX);
    if (dy!=0) dy = std::max(std::min(dy,  MAX_DYZ), -MAX_DYZ);
    if (dz!=0) dz = std::max(std::min(dz,  MAX_DYZ), -MAX_DYZ);

    Eigen::VectorXd q = q_right_state_;
    hand_func_R.HO_FK_palm(q);
    Eigen::Vector3d r0 = hand_func_R.r_palm;
    Eigen::Vector3d goal = r0 + Eigen::Vector3d(dx,dy,dz);
    Eigen::Matrix3d Rg = hand_func_R.rot(2, -100.0*M_PI/180.0, 3);

    auto pub = [&](const Eigen::VectorXd& qr){
        Eigen::VectorXd ql = Eigen::VectorXd::Zero(7);
        // Eigen::Vector3d head_angles(0,0,0);
        head_follow_hand(RIGHT, qr);
        Eigen::Vector3d head_angles(h_roll, -h_pitch, -h_yaw);
        publishMotorData(qr, ql, head_angles);
    };

    if (!moveRelative(hand_func_R, coef_generator, q,
                      Eigen::Vector3d(dx,dy,dz), goal, Rg,
                      STEP_T, T, pub))
    {
        ROS_WARN("keyboard step failed");
        return;
    }

    q_right_state_ = q;
    hand_keyboard_last_input_ = ros::WallTime::now();
}



bool HandManager::move_hand_keyboard_handler(hand_planner::KeyboardJog::Request &req,
                                             hand_planner::KeyboardJog::Response &res)
{
    Eigen::VectorXd q(7);
    if (q_right_state_.size()==7) q = q_right_state_;
    else q << 10*M_PI/180.0, -10*M_PI/180.0, 0, -25*M_PI/180.0, 0, 0, 0;
    Eigen::Vector3d mid(0.15, -0.15, -0.30);
    Eigen::Vector3d goal(0.35, -0.10, -0.20);
    Eigen::Matrix3d Rg = hand_func_R.rot(2, -100.0*M_PI/180.0, 3);

    auto pub = [&](const Eigen::VectorXd& qr){
        Eigen::VectorXd ql = Eigen::VectorXd::Zero(7);
        // Eigen::Vector3d head_angles(0,0,0);
        head_follow_hand(RIGHT, qr);
        Eigen::Vector3d head_angles(h_roll, -h_pitch, -h_yaw);
        publishMotorData(qr, ql, head_angles);
    };

    if (!approachViaOneMid(hand_func_R, coef_generator, q, mid, goal, Rg, 3.0, 3.0, T, pub)) {
        res.ok=false; res.message="approach failed";
        return true;
    }

    q_right_state_ = q;

    hand_keyboard_enabled_ = true;
    hand_keyboard_last_input_ = ros::WallTime::now();
    res.ok = true; 
    res.message = "ready: use keyboard teleop (r/f/t/g/y/h)";
    return true;
}

bool HandManager::move_hand_general_handler(hand_planner::MoveHandGeneral::Request &req,
                                            hand_planner::MoveHandGeneral::Response &res)
{
    using Eigen::Vector3d;
    using Eigen::VectorXd;
    using Eigen::Matrix3d;

    VectorXd q(7);
    if (q_right_state_.size() == 7) {
        q = q_right_state_;
    } else {
        q.resize(7);
        q << 10.0*M_PI/180.0, -10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    }

    if (q_right_baseline_.size() != 7) {
        q_right_baseline_ = q;
    }

    auto pub = [&](const VectorXd& q_abs){
        VectorXd q_send = q_abs;
        q_send.head(4) = q_abs.head(4) - q_right_baseline_.head(4);
        VectorXd ql = VectorXd::Zero(7);
        publishMotorData(q_send, ql, Vector3d(0,0,0));
    };

    const double T1 = 3.0, T2 = 3.0;
    bool user_exit = false;

    auto trim = [](std::string &s){
        while (!s.empty() && std::isspace((unsigned char)s.back())) s.pop_back();
        size_t i=0; while (i<s.size() && std::isspace((unsigned char)s[i])) ++i;
        s = s.substr(i);
    };

    for (const std::string& raw : req.commands) {
        std::string line = raw;
        trim(line);
        if (line.empty()) continue;
        if (line.size()==1 && (line[0]=='x' || line[0]=='X')) { user_exit = true; break; }

        std::istringstream iss(line);
        std::string mode; 
        if (!(iss >> mode)) continue;

        std::vector<double> vals; 
        double tmp; 
        while (iss >> tmp) vals.push_back(tmp);

        bool is_abs = (mode=="abs" || mode=="ABS");

        double mx=0, my=0, mz=0, gx=0, gy=0, gz=0, rx=0, ry=0, rz=0;
        bool have_mid = false;

        if (is_abs) {
            if (vals.size() == 9) {
                have_mid = true;
                mx = vals[0]; my = vals[1]; mz = vals[2];
                gx = vals[3]; gy = vals[4]; gz = vals[5];
                rx = vals[6]; ry = vals[7]; rz = vals[8];
            } else if (vals.size() == 6) {
                have_mid = false;
                gx = vals[0]; gy = vals[1]; gz = vals[2];
                rx = vals[3]; ry = vals[4]; rz = vals[5];
            } else {
                continue;
            }
        } else {
            if (vals.size() != 6) continue;
            gx = vals[0]; gy = vals[1]; gz = vals[2];
            rx = vals[3]; ry = vals[4]; rz = vals[5];
        }

        hand_func_R.HO_FK_palm(q);
        Vector3d r0 = hand_func_R.r_palm;
        Matrix3d R0 = hand_func_R.R_palm.block<3,3>(0,0);

        const double RX = rx * M_PI/180.0;
        const double RY = ry * M_PI/180.0;
        const double RZ = rz * M_PI/180.0;

        Matrix3d R_inc = hand_func_R.rot(2, RY, 3)
                       * hand_func_R.rot(1, RX, 3)
                       * hand_func_R.rot(3, RZ, 3);

        Vector3d r_goal, mid;
        Matrix3d R_goal;

        if (is_abs) {
            r_goal = Vector3d(gx, gy, gz);
            mid    = have_mid ? Vector3d(mx, my, mz) : 0.5*(r0 + r_goal);
            R_goal = R_inc;                
        } else {
            r_goal = r0 + Vector3d(gx, gy, gz);
            mid    = 0.5*(r0 + r_goal);   
            R_goal = R0 * R_inc;          
        }

        if (!approachViaOneMid(hand_func_R, coef_generator, q, mid, r_goal, R_goal, T1, T2, T, pub)) {
            res.ok = false; 
            res.message = "approach failed";
            q_right_state_ = q;
            return true;
        }
    }

    q_right_state_ = q;
    res.ok = true; 
    res.message = user_exit ? "exit" : "done";
    return true;
}

bool HandManager::arm_back_to_home_handler(hand_planner::arm_back_to_home::Request &req,
                                           hand_planner::arm_back_to_home::Response &res)
{
    ros::Rate rate_(rate);

    // Right init
    Eigen::VectorXd q_r(7);
    if (q_right_state_.size()==7) q_r = q_right_state_;
    else q_r << 10.0*M_PI/180.0, -10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    if (q_right_baseline_.size()!=7) q_right_baseline_ = q_r;

    Eigen::VectorXd q_target_r = q_r;
    for (int i=0;i<4;++i) q_target_r(i) = q_right_baseline_(i);
    q_target_r(4) = 0.0;
    q_target_r(5) = q_r(5);
    q_target_r(6) = q_r(6);

    // Left init
    Eigen::VectorXd q_l(7);
    if (q_left_state_.size()==7) q_l = q_left_state_;
    else q_l << 10.0*M_PI/180.0, 10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    if (q_left_baseline_.size()!=7) q_left_baseline_ = q_l;

    Eigen::VectorXd q_target_l = q_l;
    for (int i=0;i<4;++i) q_target_l(i) = q_left_baseline_(i);
    q_target_l(4) = 0.0;
    q_target_l(5) = q_l(5);
    q_target_l(6) = q_l(6);

    // Params
    const int NUM_ARM_JOINTS_TO_HOME = 5;
    const double FIXED_HOME_DURATION = 15.0;
    double segment_duration = FIXED_HOME_DURATION / NUM_ARM_JOINTS_TO_HOME;
    if (segment_duration <= 0.0) segment_duration = T;
    int M_segment = static_cast<int>(segment_duration / T);
    if (M_segment == 0) M_segment = 1;

    Eigen::VectorXd q_work_r = q_r;
    Eigen::VectorXd q_work_l = q_l;

    auto pub = [&](const Eigen::VectorXd& qr,const Eigen::VectorXd& ql){
        Eigen::VectorXd q_send_r = qr;
        q_send_r.head(4) = qr.head(4) - q_right_baseline_.head(4);
        Eigen::VectorXd q_send_l = ql;
        q_send_l.head(4) = ql.head(4) - q_left_baseline_.head(4);
        publishMotorData(q_send_r, q_send_l, Eigen::Vector3d(0,0,0));
    };

    for (int joint_idx = NUM_ARM_JOINTS_TO_HOME - 1; joint_idx >= 0; --joint_idx) {
        double start_r = q_work_r(joint_idx);
        double end_r   = q_target_r(joint_idx);
        double start_l = q_work_l(joint_idx);
        double end_l   = q_target_l(joint_idx);

        for (int step = 0; step <= M_segment; ++step) {
            double a = (M_segment > 0) ? static_cast<double>(step)/M_segment : 1.0;
            q_work_r(joint_idx) = start_r + (end_r - start_r)*a;
            q_work_l(joint_idx) = start_l + (end_l - start_l)*a;
            pub(q_work_r,q_work_l);
            ros::spinOnce();
            rate_.sleep();
        }
    }

    q_right_state_ = q_work_r;
    q_left_state_  = q_work_l;
    res.success = true;
    return true;
}

bool HandManager::fingerControlService(hand_planner::FingerControl::Request &req, hand_planner::FingerControl::Response &res) {
    ROS_INFO("Received direct finger control request with hand selection: %s", req.hand_selection.c_str());
    
    try {
        // Convert hand selection to enum (default to RIGHT_HAND if not specified)
        HandSelection hand = (req.hand_selection.empty()) ? HandSelection::RIGHT_HAND : finger_control_->stringToHandSelection(req.hand_selection);
        
        // Convert request data to vectors
        std::vector<uint8_t> positions(req.target_positions.begin(), req.target_positions.end());
        std::vector<uint8_t> limits(req.pressure_limits.begin(), req.pressure_limits.end());
        global_finger_trigger = 1;
        
        bool success = finger_control_->setDirectControl(positions, limits, req.pid_kp, req.pid_ki, req.pid_kd, hand);
        
        if (success) {
            res.success = true;
            res.message = "Direct finger control parameters set successfully";
        } else {
            res.success = false;
            res.message = "Failed to set direct finger control parameters";
        }
        
    } catch (const std::exception& e) {
        res.success = false;
        res.message = "Exception in direct finger control: " + std::string(e.what());
    }
    global_finger_trigger = 0;
    return true;
}

bool HandManager::fingerScenarioService(hand_planner::FingerScenario::Request &req, hand_planner::FingerScenario::Response &res) {
    ROS_INFO("Received finger scenario request for scenario: %s with hand selection: %s", req.scenario_name.c_str(), req.hand_selection.c_str());
    
    try {
        // Convert hand selection to enum (default to RIGHT_HAND if not specified)
        HandSelection hand = (req.hand_selection.empty()) ? HandSelection::RIGHT_HAND : finger_control_->stringToHandSelection(req.hand_selection);
        global_finger_trigger = 1;
        
        if (finger_control_->executeScenario(req.scenario_name, hand)) {
            res.success = true;
            res.message = "Scenario executed successfully";
        } else {
            res.success = false;
            res.message = "Failed to execute scenario";
        }
        
    } catch (const std::exception& e) {
        res.success = false;
        res.message = "Exception in finger scenario: " + std::string(e.what());
    }
    global_finger_trigger = 0;
    return true;
}


// // --- Main Function ---
// int main(int argc, char **argv) {
//     ros::init(argc, argv, "hand_manager_node");
//     ros::NodeHandle n;
//     HandManager node_handler(&n);
//     ROS_INFO("Hand Manager Node is ready to receive service calls.");
//     ros::spin();
//     return 0;
// }