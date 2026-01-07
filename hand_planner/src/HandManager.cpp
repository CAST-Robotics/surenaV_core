#include "HandManager.h"

// --- CONSTRUCTOR ---
HandManager::HandManager(ros::NodeHandle *n) : 
    hand_func_R(RIGHT), 
    hand_func_L(LEFT),

    // Initialize parameters
    T(0.005),
    rate(200),
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

    sendHandMotorCommands(VectorXd::Zero(7), VectorXd::Zero(7), Vector3d::Zero());
    sendHeadMotorCommands(Vector3d::Zero());

    // ROS Communication Setup
    publish_trigger_pub_         = n->advertise<std_msgs::Empty>("robot_manager/publish_trigger", 1);
    camera_data_sub              = n->subscribe("/detection_info", 1, &HandManager::object_detect_callback, this);
    joint_qc_sub                 = n->subscribe("jointdata/qc", 100, &HandManager::joint_qc_callback, this);
    teleoperation_data_sub       = n->subscribe("teleoperation/angles", 100, &HandManager::teleoperation_callback, this);
    micArray_data_sub            = n->subscribe("micarray/angle", 100, &HandManager::micArray_callback, this);
    hall_sensor_sub_             = n->subscribe("/surena/hall_state", 1, &HandManager::hallSensorCallback, this);
    hand_keyboard_sub_           = n->subscribe("/keyboard_command", 10, &HandManager::hand_keyboard_callback, this);
    head_keyboard_sub_           = n->subscribe("/keyboard_command", 10, &HandManager::head_keyboard_callback, this);
    move_hand_single_service     = n->advertiseService("move_hand_single_srv", &HandManager::single_hand, this);
    move_hand_both_service       = n->advertiseService("move_hand_both_srv", &HandManager::both_hands, this);
    grip_online_service          = n->advertiseService("grip_online_srv", &HandManager::grip_online, this);
    set_target_class_service     = n->advertiseService("set_target_class_srv", &HandManager::setTargetClassService, this);
    head_track_service           = n->advertiseService("head_track_srv", &HandManager::head_track_handler, this);
    teleoperation_service        = n->advertiseService("teleoperation_srv", &HandManager::teleoperation_handler, this);
    write_string_service_        = n->advertiseService("write_string_srv", &HandManager::write_string_handler, this);
    move_hand_relative_service_  = n->advertiseService("move_hand_relative_srv", &HandManager::move_hand_relative_handler, this);
    move_hand_keyboard_service_  = n->advertiseService("move_hand_keyboard_srv", &HandManager::move_hand_keyboard_handler, this);
    move_head_keyboard_service_  = n->advertiseService("move_head_keyboard_srv", &HandManager::move_head_keyboard_handler, this);
    move_hand_general_service_   = n->advertiseService("move_hand_general_srv", &HandManager::move_hand_general_handler, this);
    move_hand_general_left_service_   = n->advertiseService("move_hand_general_left_srv", &HandManager::move_hand_general_left_handler, this);
    move_hands_general_srv_ = n->advertiseService("move_hands_general_srv", &HandManager::move_hands_general_handler, this);
    arm_back_to_home_service_    = n->advertiseService("arm_back_to_home_srv", &HandManager::arm_back_to_home_handler, this);
    finger_control_service_      = n->advertiseService("finger_control_srv", &HandManager::fingerControlService, this);
    finger_scenario_service_     = n->advertiseService("finger_scenario_srv", &HandManager::fingerScenarioService, this);
    arm_home_service_            = n->advertiseService("arm_home_service", &HandManager::arm_home_service_handler, this);
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

const std::vector<double>& HandManager::getHandMotorCommands() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return hand_motor_commands_;
}

const std::vector<double>& HandManager::getHandGazeboCommands() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return hand_gazebo_commands_;
}

const std::vector<double>& HandManager::getHeadMotorCommands() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return head_motor_commands_;
}

const std::vector<double>& HandManager::getHeadGazeboCommands() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return head_gazebo_commands_;
}

const std::vector<double>& HandManager::getFingerMotorCommands() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return finger_control_-> getFingerCommands();
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

void HandManager::hallSensorCallback(const std_msgs::Int32& msg){
    if (!(msg.data &(1<<8))){      // Sensor 1: RIght Elbow
        hall_sensors_state[0] = 1;
    } 
    if (!(msg.data &(1<<9))) {     // Sensor 2: Right Yaw
        hall_sensors_state[1] = 1;
    }  
    if (!(msg.data &(1<<10))) {    // Sensor 3: Right Roll
        hall_sensors_state[2] = 1;
    }  
    if (!(msg.data &(1<<11))) {    // Sensor 4: Right Pitch
        hall_sensors_state[3] = 1;
    }  
    if (!(msg.data &(1<<12))) {    // Sensor 5: Left Elbow
        hall_sensors_state[4] = 1;
    }  
    if (!(msg.data &(1<<13))) {    // Sensor 6: Left Yaw
        hall_sensors_state[5] = 1;
    }  
    if (!(msg.data &(1<<14))) {    // Sensor 7: Left Roll
        hall_sensors_state[6] = 1;
    }  
    if (!(msg.data &(1<<3))) {     // Sensor 8: Left Pitch
        hall_sensors_state[7] = 1;
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

    if (scenario == "shakeHands") {  //t=9s
        r_middle = (type == RIGHT) ? Vector3d(0.35, -0.1, -0.2) : Vector3d(0.35, 0.1, -0.2);
        r_target = (type == RIGHT) ? Vector3d(0.3, -0.03, -0.3) : Vector3d(0.3, 0.03, -0.3);
        R_target = hand_func.rot(2, -65 * M_PI / 180, 3);
    } else if (scenario == "respect") {
        r_middle = (type == RIGHT) ? Vector3d(0.3, -0.1, -0.3) : Vector3d(0.3, 0.1, -0.3);
        r_target = (type == RIGHT) ? Vector3d(0.25, 0.2, -0.3) : Vector3d(0.25, -0.2, -0.3);
        double rot_angle = (type == RIGHT) ? 60 * M_PI / 180 : -60 * M_PI / 180;
        R_target = hand_func.rot(2, -80 * M_PI / 180, 3) * hand_func.rot(1, rot_angle, 3); //-130 and -70 lefthand
    } else if (scenario == "byebye") { //t=8s
        r_middle = (type == RIGHT) ? Vector3d(0.3, -0.1, 0.0) : Vector3d(0.35, 0.2, -0.15);
        r_target = (type == RIGHT) ? Vector3d(0.27, 0.0, 0.15) : Vector3d(0.3, 0.1, 0.22);
        R_target = hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot(1, -140 * M_PI / 180, 3);
    } else if (scenario == "punching") {
        r_middle = (type == RIGHT) ? Vector3d(0.15, -0.1, -0.1) : Vector3d(0.15, 0.1, -0.1);
        r_target = (type == RIGHT) ? Vector3d(0.35, 0.1, 0.1) : Vector3d(0.35, -0.1, 0.1);
        double rot_angle = (type == RIGHT) ? 90 * M_PI / 180 : -90 * M_PI / 180;
        R_target = hand_func.rot(3, rot_angle, 3) * hand_func.rot(1, -90 * M_PI / 180, 3);
    } else if (scenario == "perfect") {
        r_middle = (type == RIGHT) ? Vector3d(0.15, -0.1, -0.3) : Vector3d(0.15, 0.1, -0.3);
        r_target = (type == RIGHT) ? Vector3d(0.25, -0.05, -0.25) : Vector3d(0.25, 0.15, -0.15);
        double rot_angle = (type == RIGHT) ? -45 * M_PI / 180 : 45 * M_PI / 180;
        R_target = hand_func.rot(2, -90 * M_PI / 180, 3) * hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot(1, rot_angle, 3);
    } else if (scenario == "pointing") {
        r_middle = (type == RIGHT) ? Vector3d(0.25, -0.1, -0.1) : Vector3d(0.25, 0.1, -0.1);
        r_target = (type == RIGHT) ? Vector3d(0.45, 0.05, 0.0) : Vector3d(0.45, -0.05, 0.0); // fix left hand
        double rot_angle = (type == RIGHT) ? 90 * M_PI / 180 : -90 * M_PI / 180;
        R_target = hand_func.rot(3, rot_angle, 3) * hand_func.rot(1, -65 * M_PI / 180, 3);
    } else if (scenario == "like") {
        r_middle = (type == RIGHT) ? Vector3d(0.15, -0.1, -0.3) : Vector3d(0.15, 0.1, -0.3);
        r_target = (type == RIGHT) ? Vector3d(0.25, -0.05, -0.25) : Vector3d(0.25, 0.05, -0.25);
        R_target = hand_func.rot(2, -90 * M_PI / 180, 3);
    } else if (scenario == "showHands") {
        r_middle = (type == RIGHT) ? Vector3d(0.2, 0, -0.25) : Vector3d(0.1, 0, -0.35);
        r_target = (type == RIGHT) ? Vector3d(0.35, 0, -0.1) : Vector3d(0.25, -0.05, -0.2);
        R_target = (type == RIGHT) ? hand_func.rot(2, -130 * M_PI / 180, 3)*hand_func.rot(3, -90 * M_PI / 180, 3):hand_func.rot(2, -130 * M_PI / 180, 3)*hand_func.rot(1, -70 * M_PI / 180, 3) ;
    } else if (scenario == "home") {
        r_middle = (type == RIGHT) ? Vector3d(0.2, -0.07, -0.25) : Vector3d(0.2, 0.07, -0.25);
        r_target = (type == RIGHT) ? Vector3d(0.02, -0.06, -0.46) : Vector3d(0.02, 0.06, -0.46);
        R_target = hand_func.rot(2, -20 * M_PI / 180, 3);
    }else if (scenario == "a") {
        r_middle = (type == RIGHT) ? Vector3d(0.15, 0.0, -0.1) : Vector3d(0.25, 0.0, -0.15);
        r_target = (type == RIGHT) ? Vector3d(0.2, 0.0, -0.15) : Vector3d(0.25, 0.0, -0.15);
        double rot_angle = (type == RIGHT) ? 0 * M_PI / 180 : 0 * M_PI / 180;
        R_target = hand_func.rot(2, -60 * M_PI / 180, 3) * hand_func.rot(1, rot_angle, 3);
    } else if (scenario == "b") {
        r_middle = (type == RIGHT) ? Vector3d(0.2, 0.0, -0.2) : Vector3d(0.35, -0.05, -0.2);
        r_target = (type == RIGHT) ? Vector3d(0.25, 0.1, -0.2) : Vector3d(0.35, -0.1, -0.2);
        double rot_angle = (type == RIGHT) ? 20 * M_PI / 180 : -20 * M_PI / 180;
        R_target = hand_func.rot(2, -80 * M_PI / 180, 3) * hand_func.rot(1, rot_angle, 3); 
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

    Vector3d r_middle = targets.row(0);
    Vector3d r_target = targets.row(1);
    Vector3d r_start  = targets.row(5);
    Matrix3d R_target = targets.block(2, 0, 3, 3);

    MatrixXd t_r(1,3); t_r << 0, 2, 4;
    const double t_reach_end = 4.0;

    MatrixXd P_x(1,3), P_y(1,3), P_z(1,3);
    P_x << r_start(0), r_middle(0), r_target(0);
    P_y << r_start(1), r_middle(1), r_target(1);
    P_z << r_start(2), r_middle(2), r_target(2);

    MatrixXd V_inf(1,3); V_inf << 0, INFINITY, 0;
    MatrixXd A_inf(1,3); A_inf << 0, INFINITY, 0;

    MatrixXd X_coef = coef_generator.Coefficient(t_r, P_x, V_inf, A_inf);
    MatrixXd Y_coef = coef_generator.Coefficient(t_r, P_y, V_inf, A_inf);
    MatrixXd Z_coef = coef_generator.Coefficient(t_r, P_z, V_inf, A_inf);

    const double T_BYE     = 4.0;
    const double AMP_BYE   = 15 * M_PI / 180.0;
    const double OMEGA_BYE = M_PI * 1.5;

    VectorXd q_frozen = VectorXd::Zero(7);
    bool freeze_ready = false;


    const double T_DWELL = 1.0;
    const double T_SHAKE = 4.0;
    const double t_shake_start = t_reach_end + T_DWELL;

    const double AMP_J3 = 10 * M_PI / 180.0;
    const double OMEGA  = M_PI;

    bool   shake_ready = false;
    double q3_base     = 0.0;


    double total_time = t_reach_end;
    if (scenario == "byebye")
        total_time += T_BYE;
    else if (scenario == "shakeHands")
        total_time += T_DWELL + T_SHAKE;

    int step_count = static_cast<int>(total_time / T);


    for (int count = 0; count < step_count; ++count) {
        double time_r = count * T;
        VectorXd q_next = VectorXd::Zero(7);

        if (time_r < t_reach_end) {

            const int interval = (time_r < t_r(1)) ? 0 : 1;
            const double t_int = (interval == 0) ? t_r(0) : t_r(1);

            Vector3d V_curr;
            V_curr <<
                coef_generator.GetAccVelPos(X_coef.row(interval), time_r, t_int, 5)(0,1),
                coef_generator.GetAccVelPos(Y_coef.row(interval), time_r, t_int, 5)(0,1),
                coef_generator.GetAccVelPos(Z_coef.row(interval), time_r, t_int, 5)(0,1);

            hand_model.update_hand(q_arm, V_curr, r_target, R_target);
            hand_model.doQP(q_arm);
            q_next = hand_model.q_next;
        }
        else {

            if (scenario == "byebye") {
                if (!freeze_ready) {
                    q_frozen = q_arm;      // latch once at 4s
                    freeze_ready = true;
                }

                q_next = q_frozen;
                double t_local = time_r - t_reach_end;
                q_next(2) = q_frozen(2) + AMP_BYE * std::sin(OMEGA_BYE * t_local);
            }

            else if (scenario == "shakeHands") {

                hand_model.update_hand(q_arm, Vector3d::Zero(), r_target, R_target);
                hand_model.doQP(q_arm);
                q_next = hand_model.q_next;

                if (time_r >= t_shake_start) {
                    if (!shake_ready) {
                        q3_base = q_next(3);
                        shake_ready = true;
                    }

                    double t_local = time_r - t_shake_start;
                    if (t_local <= T_SHAKE + 1e-9) {
                        q_next(3) = q3_base - AMP_J3 * std::sin(OMEGA * (t_local + 1));
                    } else {
                        q_next(3) = q3_base;
                    }
                }
            }
            else {
                hand_model.update_hand(q_arm, Vector3d::Zero(), r_target, R_target);
                hand_model.doQP(q_arm);
                q_next = hand_model.q_next;
            }
        }

        q_arm = q_next;

        const int global_index = static_cast<int>(count + sum_arm / T);
        if (global_index < qref_arm.cols()) {
            qref_arm.col(global_index) = q_arm - q_init_arm;
        }
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

void HandManager::sendHandMotorCommands(const VectorXd& q_rad_right, const VectorXd& q_rad_left, const Vector3d& head_angles) {
    
    std::lock_guard<std::mutex> lock(command_mutex_); // Acquire lock
    std::vector<double> q_motor_temp(29, 0); 
    std::vector<double> q_gazebo_temp(29, 0.0);

    // Right hand motors (indices 12-15)
    q_motor_temp[12] = int(q_rad_right(0) * encoderResolution[0] * harmonicRatio[0] / (2 * M_PI));
    q_motor_temp[13] = -int(q_rad_right(1) * encoderResolution[0] * harmonicRatio[1] / (2 * M_PI));
    q_motor_temp[14] = int(q_rad_right(2) * encoderResolution[1] * harmonicRatio[2] / (2 * M_PI));
    q_motor_temp[15] = -int(q_rad_right(3) * encoderResolution[1] * harmonicRatio[3] / (2 * M_PI));

    // Left hand motors (indices 16-19)
    q_motor_temp[16] = -int(q_rad_left(0) * encoderResolution[0] * harmonicRatio[0] / (2 * M_PI));
    q_motor_temp[17] = -int(q_rad_left(1) * encoderResolution[0] * harmonicRatio[1] / (2 * M_PI));
    q_motor_temp[18] = int(q_rad_left(2) * encoderResolution[1] * harmonicRatio[2] / (2 * M_PI));
    q_motor_temp[19] = int(q_rad_left(3) * encoderResolution[1] * harmonicRatio[3] / (2 * M_PI));

    // Head motors (indices 20-22) - roll, pitch, yaw
    q_motor_temp[20] = int(roll_command_range[0] + (roll_command_range[1] - roll_command_range[0]) *
                        ((-(head_angles(0)*180/M_PI) - roll_range[0]) / (roll_range[1] - roll_range[0])));
    q_motor_temp[21] = int(pitch_command_range[0] + (pitch_command_range[1] - pitch_command_range[0]) *
                        ((-(head_angles(1)*180/M_PI) - pitch_range[0]) / (pitch_range[1] - pitch_range[0])));
    q_motor_temp[22] = int(yaw_command_range[0] + (yaw_command_range[1] - yaw_command_range[0]) *
                        ((-(head_angles(2)*180/M_PI) - yaw_range[0]) / (yaw_range[1] - yaw_range[0])));

    // Wrist calculations for right hand (indices 23-25)
    q_motor_temp[23] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) *
                        (((q_rad_right(4) * 180 / M_PI) - wrist_yaw_range[0]) / (wrist_yaw_range[1] - wrist_yaw_range[0])));
    // q_motor_temp[24] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) *
    //                     (((hand_func_R.wrist_right_calc(q_rad_right(5), q_rad_right(6))) - (wrist_right_range[0])) / (wrist_right_range[1] - (wrist_right_range[0]))));
    // q_motor_temp[25] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) *
    //                     (((hand_func_R.wrist_left_calc(q_rad_right(5), q_rad_right(6))) - wrist_left_range[0]) / (wrist_left_range[1] - wrist_left_range[0])));

    Vector2d right_wrist_res = hand_func_R.solve_wrist(-q_rad_right(5)*180/M_PI, q_rad_right(6)*180/M_PI); // (motor 25 (A), motor 24 (B))
    if(int(right_wrist_res(1)) != -1){
        q_motor_temp[24] = int(right_wrist_res(0));
        q_motor_temp[25] = int(right_wrist_res(1));
        right_wrist_res_temp = right_wrist_res;
    } else {
        q_motor_temp[24] = int(right_wrist_res_temp(0));
        q_motor_temp[25] = int(right_wrist_res_temp(1));
    }

    // Wrist calculations for left hand (indices 26-28)
    q_motor_temp[26] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) *
                        (((-q_rad_left(4) * 180 / M_PI) - wrist_yaw_range[0]) / (wrist_yaw_range[1] - wrist_yaw_range[0])));
    q_motor_temp[27] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) *
                        (((hand_func_L.wrist_right_calc(-q_rad_left(5), -q_rad_left(6))) - (wrist_right_range[0])) / (wrist_right_range[1] - (wrist_right_range[0]))));
    q_motor_temp[28] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) *
                        (((hand_func_L.wrist_left_calc(-q_rad_left(5), -q_rad_left(6))) - wrist_left_range[0]) / (wrist_left_range[1] - wrist_left_range[0])));

    // Vector2d left_wrist_res = hand_func_L.solve_wrist(q_rad_left(5)*180/M_PI, q_rad_left(6)*180/M_PI); // (motor 28 (A), motor 27 (B))
    // if(int(left_wrist_res(1)) != -1){
    //     q_motor_temp[27] = int(left_wrist_res(0));
    //     q_motor_temp[28] = int(left_wrist_res(1));
    //     left_wrist_res_temp = left_wrist_res;
    // } else {
    //     q_motor_temp[27] = int(left_wrist_res_temp(0));
    //     q_motor_temp[28] = int(left_wrist_res_temp(1));
    // }
    // cout <<q_motor_temp[26]<< ", "<<q_motor_temp[27]<<", "<<q_motor_temp[28]<<endl;

    hand_motor_commands_ = q_motor_temp; // Store for RobotManager
    last_q_motor = q_motor_temp; // Still useful for internal tracking if needed

    // Right hand joints
    q_gazebo_temp[12] = q_rad_right(0);
    q_gazebo_temp[13] = q_rad_right(1);
    q_gazebo_temp[14] = q_rad_right(2);
    q_gazebo_temp[15] = q_rad_right(3);

    // Left hand joints
    q_gazebo_temp[16] = q_rad_left(0);
    q_gazebo_temp[17] = q_rad_left(1);
    q_gazebo_temp[18] = q_rad_left(2);
    q_gazebo_temp[19] = q_rad_left(3);

    // Head joints
    q_gazebo_temp[20] = -head_angles(0); // roll
    q_gazebo_temp[21] = -head_angles(1); // pitch
    q_gazebo_temp[22] = -head_angles(2); // yaw

    // Wrist joints for right hand
    q_gazebo_temp[23] = q_rad_right(4);
    q_gazebo_temp[24] = q_rad_right(5);
    q_gazebo_temp[25] = q_rad_right(6);

    // Wrist joints for left hand
    q_gazebo_temp[26] = q_rad_left(4);
    q_gazebo_temp[27] = q_rad_left(5);
    q_gazebo_temp[28] = q_rad_left(6);

    hand_gazebo_commands_ = q_gazebo_temp; // Store for RobotManager
}


void HandManager::sendHeadMotorCommands(const Vector3d& head_angles) {
    
    std::lock_guard<std::mutex> lock(command_mutex_); // Acquire lock
    std::vector<double> q_motor_temp_head(3, 0); 
    std::vector<double> q_gazebo_temp_head(3, 0.0);


    // Head motors (indices 20-22) - roll, pitch, yaw
    q_motor_temp_head[0] = int(roll_command_range[0] + (roll_command_range[1] - roll_command_range[0]) *
                        ((-(head_angles(0)*180/M_PI) - roll_range[0]) / (roll_range[1] - roll_range[0])));
    q_motor_temp_head[1] = int(pitch_command_range[0] + (pitch_command_range[1] - pitch_command_range[0]) *
                        ((-(head_angles(1)*180/M_PI) - pitch_range[0]) / (pitch_range[1] - pitch_range[0])));
    q_motor_temp_head[2] = int(yaw_command_range[0] + (yaw_command_range[1] - yaw_command_range[0]) *
                        ((-(head_angles(2)*180/M_PI) - yaw_range[0]) / (yaw_range[1] - yaw_range[0])));


    head_motor_commands_ = q_motor_temp_head; // Store for RobotManager

    // Head joints
    q_gazebo_temp_head[0] = -head_angles(0); // roll
    q_gazebo_temp_head[1] = -head_angles(1); // pitch
    q_gazebo_temp_head[2] = -head_angles(2); // yaw


    head_gazebo_commands_ = q_gazebo_temp_head; // Store for RobotManager
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
        Eigen::VectorXd q_left(7);
        if (q_left_state_.size()==7){
            q_left = q_left_state_;
        } else {
            q_left.resize(7);
            q_left << 10.0*M_PI/180.0, 10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
        }
        if (q_left_baseline_.size() != 7) {
            q_left_baseline_ = q_left;
        }

        for (int id = 0; id < qref_r.cols(); ++id) {
            Eigen::VectorXd q_abs  = q_init_r + qref_r.col(id);
            Eigen::VectorXd q_send = q_abs;
            q_send.head(4) = q_abs.head(4) - q_right_baseline_.head(4);
            Eigen::VectorXd q_left_send = q_left;
            q_left_send.head(4) = q_left_send.head(4) - q_left_baseline_.head(4);

            // Eigen::Vector3d head_angles(0,0,0);
            // head_follow_hand(RIGHT, q_send);
            Eigen::Vector3d head_angles(h_roll, -h_pitch, -h_yaw);

            sendHandMotorCommands(q_send, q_left_send, head_angles);
            publish_trigger_pub_.publish(std_msgs::Empty());
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
        //q_left_baseline_ = q_left_state_;

        if (q_left_baseline_.size() != 7) q_left_baseline_ = q_init_l;

        hand_func_L.HO_FK_palm(q_la);
        next_ini_ee_posL = hand_func_L.r_palm;

        for (int i = 0; i < req.scen_count; i++) {
            Eigen::MatrixXd result = scenario_target(LEFT, req.scenario[i], i, ee_pos, "prev");
            ee_pos = reach_target(hand_func_L, q_la, qref_l, sum_l, q_init_l, result, req.scenario[i], M);
        }

        Eigen::VectorXd q_right(7);
        if (q_right_state_.size()==7) {
            q_right = q_right_state_;
        } else {
            q_right.resize(7);
            q_right << 10.0*M_PI/180.0, -10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
        }

        if (q_right_baseline_.size() != 7) {
            q_right_baseline_ = q_right;
        }


        for (int id = 0; id < qref_l.cols(); ++id) {
            Eigen::VectorXd q_abs_l  = q_init_l + qref_l.col(id);
            Eigen::VectorXd q_send_l = q_abs_l;
            q_send_l.head(4) = q_abs_l.head(4) - q_left_baseline_.head(4);

            Eigen::VectorXd q_right_send = q_right;
            q_right_send.head(4) = q_right_send.head(4) - q_right_baseline_.head(4);

            // Eigen::Vector3d head_angles(0,0,0);
            // head_follow_hand(LEFT, q_send_l);
            Eigen::Vector3d head_angles(h_roll, -h_pitch, -h_yaw);

            sendHandMotorCommands(q_right_send, q_send_l, head_angles);
            sendHeadMotorCommands(head_angles);
            publish_trigger_pub_.publish(std_msgs::Empty());
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
    q_left_baseline_ = q_left_state_;
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
        sendHandMotorCommands(q_send_r, q_send_l, head_angles);
        publish_trigger_pub_.publish(std_msgs::Empty());
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
    while (t_grip <= (60)) {
        Vector3d target2camera(X, Y, Z);
        MatrixXd T_CAM2SH = hand_func_R.ObjToNeck(-h_pitch, h_roll, -h_yaw);
        Vector3d target2shoulder = T_CAM2SH.block(0, 3, 3, 1) + T_CAM2SH.block(0, 0, 3, 3) * target2camera;
        
        // Head Pitch
        if (abs(target2camera(2)) > 0.03) {
            h_pitch += Kp * atan2(target2camera(2), sqrt(pow(target2camera(1),2) + pow(target2camera(0),2)));
            h_pitch = max(-28.0*M_PI/180, min(28.0*M_PI/180, h_pitch));
        }
        // Head Yaw
        if (abs(target2camera(1)) > 0.03) {
            h_yaw += Ky * atan2(target2camera(1), target2camera(0));
            h_yaw = max(-60.0*M_PI/180, min(60.0*M_PI/180, h_yaw));
        }

        if (t_grip >= 15) {
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
        sendHandMotorCommands(q_delta, q_rad_left, head_angles);
        publish_trigger_pub_.publish(std_msgs::Empty());
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
            if (abs(target2camera(2)) > 0.03) {
                h_pitch += Kp * atan2(target2camera(2), sqrt(pow(target2camera(1), 2) + pow(target2camera(0), 2)));
                h_pitch = max(-28.0 * M_PI / 180, min(28.0 * M_PI / 180, h_pitch));
            }
            // Head Yaw
            if (abs(target2camera(1)) > 0.03) {
                h_yaw += Ky * atan2(target2camera(1), target2camera(0));
                h_yaw = max(-60.0 * M_PI / 180, min(60.0 * M_PI / 180, h_yaw));
            }
        } else { // follow direction of arrival (voice) instead.
            h_yaw += Ky * micArray_theta;
            h_yaw = max(-60.0 * M_PI / 180, min(60.0 * M_PI / 180, h_yaw));
        }
        
        Eigen::VectorXd q_left(7);
        if (q_left_state_.size()==7){
            q_left = q_left_state_;
        } else {
            q_left.resize(7);
            q_left << 10.0*M_PI/180.0, 10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
        }
        if (q_left_baseline_.size() != 7) {
            q_left_baseline_ = q_left;
        }

        Eigen::VectorXd q_right(7);
        if (q_right_state_.size()==7){
            q_right = q_right_state_;
        } else {
            q_right.resize(7);
            q_right << 10.0*M_PI/180.0, -10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
        }
        if (q_right_baseline_.size() != 7) {
            q_right_baseline_ = q_right;
        }

        Eigen::VectorXd q_right_send = q_right;
        q_right_send.head(4) = q_right_send.head(4) - q_right_baseline_.head(4);
        
        Eigen::VectorXd q_left_send = q_left;
        q_left_send.head(4) = q_left_send.head(4) - q_left_baseline_.head(4);

        Vector3d head_angles(h_roll, h_pitch, h_yaw);
        sendHeadMotorCommands(head_angles);
        sendHandMotorCommands(q_right_send, q_left_send, head_angles);
        publish_trigger_pub_.publish(std_msgs::Empty());
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
        sendHandMotorCommands(q_rad_right, q_rad_left, head_angles);
        publish_trigger_pub_.publish(std_msgs::Empty());
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
    Eigen::VectorXd q_left(7);
    if (q_left_state_.size()==7) {
        q_left = q_left_state_;
    } else {
        q_left.resize(7);
        q_left << 10.0*M_PI/180.0, 10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    }
    if (q_left_baseline_.size() != 7) {
        q_left_baseline_ = q_left;
    }
    auto pub = [&](const VectorXd& q_abs){
        VectorXd q_send = q_abs;
        q_send.head(4) = q_abs.head(4) - q_right_baseline_.head(4);
        Eigen::VectorXd q_left_send = q_left;
        q_left_send.head(4) = q_left_send.head(4) - q_left_baseline_.head(4);
        sendHandMotorCommands(q_send, q_left_send, Vector3d(0,0,0));
        publish_trigger_pub_.publish(std_msgs::Empty());
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
    Eigen::VectorXd q_left(7);
    if (q_left_state_.size()==7) {
        q_left = q_left_state_;
    } else {
        q_left.resize(7);
        q_left << 10.0*M_PI/180.0, 10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    }
    if (q_left_baseline_.size() != 7) {
        q_left_baseline_ = q_left;
    }

    auto pub = [&](const VectorXd& q_abs){
        VectorXd q_send = q_abs;
        q_send.head(4) = q_abs.head(4) - q_right_baseline_.head(4);
        Eigen::VectorXd q_left_send = q_left;
        q_left_send.head(4) = q_left_send.head(4) - q_left_baseline_.head(4);
        sendHandMotorCommands(q_send, q_left_send, Vector3d(0,0,0));
        publish_trigger_pub_.publish(std_msgs::Empty());
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
    if (!hand_keyboard_enabled_ || !isHandKeyboardTrajectoryEnabled) return;
    
    const double STEP_T = 1.0;
    const double MAX_DX = 0.08, MAX_DYZ = 0.30;

    int code = msg->data;
    
    // Handle ESC key to stop the service
    if (code == 27) { // ESC key
        isHandKeyboardTrajectoryEnabled = false;
        isHandKeyboardActive = false;
        ROS_INFO("Hand keyboard control stopped by ESC key");
        return;
    }
    
    double dx=0, dy=0, dz=0;
    switch (code) {
        case 't': case 'T': dx = +0.05; break; // Forward
        case 'g': case 'G': dx = -0.05; break; // Backward
        case 'f': case 'F': dy = +0.10; break; // To the left
        case 'h': case 'H': dy = -0.10; break; // To the right
        case 'i': case 'I': dz = +0.10; break; // Upward
        case 'k': case 'K': dz = -0.10; break; // Downward
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
    Eigen::VectorXd q_left(7);
    if (q_left_state_.size()==7)
    { q_left = q_left_state_;
    } else {
        q_left.resize(7);
        q_left << 10.0*M_PI/180.0, 10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    }
    if (q_left_baseline_.size() != 7) {
        q_left_baseline_ = q_left;
    }
    auto pub = [&](const Eigen::VectorXd& qr){
        Eigen::VectorXd q_left_send = q_left;
        q_left_send.head(4) = q_left_send.head(4) - q_left_baseline_.head(4);
        // Eigen::Vector3d head_angles(0,0,0);
        // head_follow_hand(RIGHT, qr);
        Eigen::Vector3d head_angles(h_roll, -h_pitch, -h_yaw);
        sendHandMotorCommands(qr, q_left_send, head_angles);
        publish_trigger_pub_.publish(std_msgs::Empty());
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


void HandManager::head_keyboard_callback(const std_msgs::Int32::ConstPtr& msg)
{
    if (!isHeadKeyboardActive) {
        return;
    }
    h_pitch_start_actual = 0.0;
    h_yaw_start_actual = 0.0;
    h_roll_start_actual = 0.0;
    h_pitch_target_set = 0.0;
    h_yaw_target_set = 0.0;
    h_roll_target_set = 0.0;
    head_motion_segment_duration = 0.9; 
    is_head_moving_segment = false;
    interpolation_step_count = 0;
    total_interpolation_steps = 0; 

    total_interpolation_steps = static_cast<int>(head_motion_segment_duration * rate);

    if (!isHeadKeyboardActive) {
        return;
    }

    const double HEAD_ANGLE_INCREMENT_DEGREES = 15.0;
    const double HEAD_ANGLE_INCREMENT_RAD = HEAD_ANGLE_INCREMENT_DEGREES * M_PI / 180.0; 

    static const double YAW_MIN_RAD   = -60.0 * M_PI / 180.0;
    static const double YAW_MAX_RAD   =  60.0 * M_PI / 180.0;
    static const double PITCH_MIN_RAD = -28.0 * M_PI / 180.0;
    static const double PITCH_MAX_RAD =  28.0 * M_PI / 180.0;
    static const double ROLL_MIN_RAD = -20.0 * M_PI / 180.0;
    static const double ROLL_MAX_RAD =  20.0 * M_PI / 180.0;

    auto clamp = [](double v, double lo, double hi){ return std::max(lo,std::min(hi,v)); };

    int key_code = msg->data; 

    if (key_code == 27) { // ASCII code for ESC key
        isHeadKeyboardActive = false;
        is_head_moving_segment = false; 
        interpolation_step_count = 0; 
        return;
    }

    h_pitch_start_actual = h_pitch;
    h_yaw_start_actual = h_yaw;
    h_roll_start_actual = h_roll;
    h_pitch_target_set = h_pitch;
    h_yaw_target_set = h_yaw;
    h_roll_target_set = h_roll;

    bool command_processed = false;

    switch (key_code) {
        case 105: // 'i'
            h_pitch_target_set += HEAD_ANGLE_INCREMENT_RAD;
            command_processed = true;
            break;
        case 107: // 'k'
            h_pitch_target_set -= HEAD_ANGLE_INCREMENT_RAD;
            command_processed = true;
            break;
        case 106: // 'j'
            h_yaw_target_set += HEAD_ANGLE_INCREMENT_RAD;
            command_processed = true;
            break;
        case 108: // 'l'
            h_yaw_target_set -= HEAD_ANGLE_INCREMENT_RAD;
            command_processed = true;
            break;
        case 117: // 'u' 
            h_roll_target_set += HEAD_ANGLE_INCREMENT_RAD - 12;
            command_processed = true;
            break;
        case 111: // 'o' (
            h_roll_target_set -= HEAD_ANGLE_INCREMENT_RAD - 12;
            command_processed = true;
            break;
        default:
            return; 
    }

    h_pitch_target_set = clamp(h_pitch_target_set, PITCH_MIN_RAD, PITCH_MAX_RAD);
    h_yaw_target_set   = clamp(h_yaw_target_set,   YAW_MIN_RAD,   YAW_MAX_RAD);
    h_roll_target_set  = clamp(h_roll_target_set,  ROLL_MIN_RAD,  ROLL_MAX_RAD);

    if (command_processed && (std::abs(h_pitch_target_set - h_pitch_start_actual) > 1e-6 || std::abs(h_yaw_target_set - h_yaw_start_actual) > 1e-6 || std::abs(h_roll_target_set - h_roll_start_actual) > 1e-6)) {
        interpolation_step_count = 0; 
        is_head_moving_segment = true;
    } else {
        is_head_moving_segment = false; 
        interpolation_step_count = 0;
    }
 
}



bool HandManager::move_head_keyboard_handler(hand_planner::headkeyboardjog::Request &req,
                                            hand_planner::headkeyboardjog::Response &res)
{

    isHeadKeyboardActive = true;
    ros::Rate loop_rate(rate); 

    h_pitch_target_set = h_pitch; 
    h_yaw_target_set = h_yaw;     
    h_roll_target_set = h_roll;
    is_head_moving_segment = false;
    interpolation_step_count = 0; 

    while (ros::ok() && isHeadKeyboardActive)
    {
        ros::spinOnce();

        if (is_head_moving_segment) {
            interpolation_step_count++;

            if (interpolation_step_count<= total_interpolation_steps) {

                double alpha = static_cast<double>(interpolation_step_count) / total_interpolation_steps;
                alpha = std::min(1.0, alpha); 

                h_pitch = h_pitch_start_actual + alpha * (h_pitch_target_set - h_pitch_start_actual);
                h_yaw   = h_yaw_start_actual   + alpha * (h_yaw_target_set   - h_yaw_start_actual);
                h_roll  = h_roll_start_actual  + alpha * (h_roll_target_set  - h_roll_start_actual);

            } else {
                h_pitch = h_pitch_target_set;
                h_yaw   = h_yaw_target_set;
                h_roll  = h_roll_target_set;

                is_head_moving_segment = false; 
                interpolation_step_count = 0; 
            }
        }

        Eigen::VectorXd q_left(7);
        if (q_left_state_.size()==7){
            q_left = q_left_state_;
        } else {
            q_left.resize(7);
            q_left << 10.0*M_PI/180.0, 10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
        }
        if (q_left_baseline_.size() != 7) {
            q_left_baseline_ = q_left;
        }

        Eigen::VectorXd q_right(7);
        if (q_right_state_.size()==7){
            q_right = q_right_state_;
        } else {
            q_right.resize(7);
            q_right << 10.0*M_PI/180.0, -10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
        }
        if (q_right_baseline_.size() != 7) {
            q_right_baseline_ = q_right;
        }

        Eigen::VectorXd q_right_send = q_right;
        q_right_send.head(4) = q_right_send.head(4) - q_right_baseline_.head(4);

        Eigen::VectorXd q_left_send = q_left;
        q_left_send.head(4) = q_left_send.head(4) - q_left_baseline_.head(4);

        // sendHandMotorCommands(q_right_send, q_left_send);
        Vector3d head_angles(h_roll, h_pitch, h_yaw);
        sendHeadMotorCommands(head_angles);
        // sendHandMotorCommands(q_right_send, q_left_send, head_angles);
        publish_trigger_pub_.publish(std_msgs::Empty());
        // cout << head_angles[0] << ", " << head_angles[1] << ", " << head_angles[2] << endl;

        loop_rate.sleep(); 
    }

    isHeadKeyboardActive = false;
    is_head_moving_segment = false;
    interpolation_step_count = 0; 
    res.success = true;
    res.message = "Head keyboard control stopped.";
    ROS_INFO("%s", res.message.c_str());
    return true;
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

    Eigen::VectorXd q_left(7);
    if (q_left_state_.size()==7){
        q_left = q_left_state_;
    } else {
        q_left.resize(7);
        q_left << 10.0*M_PI/180.0, 10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    }
    if (q_left_baseline_.size() != 7) {
        q_left_baseline_ = q_left;
    }
    auto pub = [&](const Eigen::VectorXd& qr){
        Eigen::VectorXd q_left_send = q_left;
        q_left_send.head(4) = q_left_send.head(4) - q_left_baseline_.head(4);
        // Eigen::Vector3d head_angles(0,0,0);
        // head_follow_hand(RIGHT, qr);
        Eigen::Vector3d head_angles(h_roll, -h_pitch, -h_yaw);
        sendHandMotorCommands(qr, q_left_send, head_angles);
        publish_trigger_pub_.publish(std_msgs::Empty());
    };

    if (!approachViaOneMid(hand_func_R, coef_generator, q, mid, goal, Rg, 3.0, 3.0, T, pub)) {
        res.ok=false; 
        res.message="approach failed";
        return true;
    }

    q_right_state_ = q;

    // Set up keyboard control state
    hand_keyboard_enabled_ = true;
    isHandKeyboardActive = true;
    isHandKeyboardTrajectoryEnabled = true;
    hand_keyboard_last_input_ = ros::WallTime::now();
    
    int rate = 200;
    ros::Rate rate_(rate);
    
    while (isHandKeyboardActive)
    {
        // Process any pending keyboard commands
        ros::spinOnce();
        rate_.sleep();
    }
    
    // Clean up
    hand_keyboard_enabled_ = false;
    isHandKeyboardActive = false;
    isHandKeyboardTrajectoryEnabled = false;
    
    res.ok = true; 
    res.message = "hand keyboard control stopped";
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

    Eigen::VectorXd q_left(7);
    if (q_left_state_.size()==7){
        q_left = q_left_state_;
    } else {
        q_left.resize(7);
        q_left << 10.0*M_PI/180.0, 10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    }
    if (q_left_baseline_.size() != 7) {
        q_left_baseline_ = q_left;
    }

    auto pub = [&](const VectorXd& q_abs){
        VectorXd q_send = q_abs;
        q_send.head(4) = q_abs.head(4) - q_right_baseline_.head(4);
        Eigen::VectorXd q_left_send = q_left;
        q_left_send.head(4) = q_left_send.head(4) - q_left_baseline_.head(4);
        // head_follow_hand(RIGHT, q_send);
        Eigen::Vector3d head_angles(h_roll, -h_pitch, -h_yaw);
        sendHandMotorCommands(q_send, q_left_send, head_angles);
        // sendHandMotorCommands(q_send, q_left_send, Vector3d(0,0,0));
        publish_trigger_pub_.publish(std_msgs::Empty());
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


bool HandManager::move_hand_general_left_handler(hand_planner::MoveHandGeneral::Request &req,
                                            hand_planner::MoveHandGeneral::Response &res)
{
    using Eigen::Vector3d;
    using Eigen::VectorXd;
    using Eigen::Matrix3d;

    VectorXd q(7);
    if (q_left_state_.size() == 7) {
        q = q_left_state_;
    } else {
        q.resize(7);
        q << 10.0*M_PI/180.0, 10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    }
    if (q_left_baseline_.size() != 7) {
        q_left_baseline_ = q;
    }

    Eigen::VectorXd q_right(7);
    if (q_right_state_.size()==7){
        q_right = q_right_state_;
    } else {
        q_right.resize(7);
        q_right << 10.0*M_PI/180.0, -10.0*M_PI/180.0, 0.0, -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    }
    if (q_right_baseline_.size() != 7) {
        q_right_baseline_ = q_right;
    }

    auto pub = [&](const VectorXd& q_abs){
        VectorXd q_send = q_abs;
        q_send.head(4) = q_abs.head(4) - q_left_baseline_.head(4);
        Eigen::VectorXd q_right_send = q_right;
        q_right_send.head(4) = q_right_send.head(4) - q_right_baseline_.head(4);
        // head_follow_hand(LEFT, q_send);
        Eigen::Vector3d head_angles(h_roll, -h_pitch, -h_yaw);
        sendHandMotorCommands(q_right_send, q_send, head_angles);
        // sendHandMotorCommands(q_send, q_left_send, Vector3d(0,0,0));
        publish_trigger_pub_.publish(std_msgs::Empty());
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

        hand_func_L.HO_FK_palm(q);
        Vector3d r0 = hand_func_L.r_palm;
        Matrix3d R0 = hand_func_L.R_palm.block<3,3>(0,0);

        const double RX = rx * M_PI/180.0;
        const double RY = ry * M_PI/180.0;
        const double RZ = rz * M_PI/180.0;

        Matrix3d R_inc = hand_func_L.rot(2, RY, 3)
                       * hand_func_L.rot(1, RX, 3)
                       * hand_func_L.rot(3, RZ, 3);

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

        if (!approachViaOneMid(hand_func_L, coef_generator, q, mid, r_goal, R_goal, T1, T2, T, pub)) {
            res.ok = false; 
            res.message = "approach failed";
            q_left_state_ = q;
            return true;
        }
    }

    q_left_state_ = q;
    res.ok = true; 
    res.message = user_exit ? "exit" : "done";
    return true;
}

bool HandManager::move_hands_general_handler(hand_planner::MoveHandsGeneral::Request &req,
                                             hand_planner::MoveHandsGeneral::Response &res)
{
    using Eigen::Vector3d;
    using Eigen::VectorXd;
    using Eigen::Matrix3d;

    VectorXd q_right(7);
    if (q_right_state_.size() == 7) {
        q_right = q_right_state_;
    } else {
        q_right << 10.0*M_PI/180.0, -10.0*M_PI/180.0, 0.0,
                   -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    }
    if (q_right_baseline_.size() != 7) {
        q_right_baseline_ = q_right;
    }

    VectorXd q_left(7);
    if (q_left_state_.size() == 7) {
        q_left = q_left_state_;
    } else {
        q_left << 10.0*M_PI/180.0, 10.0*M_PI/180.0, 0.0,
                  -25.0*M_PI/180.0, 0.0, 0.0, 0.0;
    }
    if (q_left_baseline_.size() != 7) {
        q_left_baseline_ = q_left;
    }

    auto trim = [](std::string &s){
        while (!s.empty() && std::isspace((unsigned char)s.back())) s.pop_back();
        size_t i = 0;
        while (i < s.size() && std::isspace((unsigned char)s[i])) ++i;
        s = s.substr(i);
    };

    const double T1 = 3.0, T2 = 3.0;

    std::atomic<bool> right_ok(true);
    std::atomic<bool> left_ok(true);
    std::atomic<bool> user_exit(false);

    std::vector<std::string> right_cmds = req.right_commands;
    std::vector<std::string> left_cmds  = req.left_commands;

    auto pub_right = [&](const VectorXd& q_abs)
    {
        std::lock_guard<std::mutex> lock(hands_mutex_);
        q_right = q_abs;

        VectorXd q_right_send = q_right;
        VectorXd q_left_send  = q_left;

        q_right_send.head(4) = q_right_send.head(4) - q_right_baseline_.head(4);
        q_left_send.head(4)  = q_left_send.head(4)  - q_left_baseline_.head(4);

        // head_follow_hand(RIGHT, q_right_send);
        Vector3d head_angles(h_roll, -h_pitch, -h_yaw);
        sendHandMotorCommands(q_right_send, q_left_send, head_angles);
        publish_trigger_pub_.publish(std_msgs::Empty());
    };

    auto pub_left = [&](const VectorXd& q_abs)
    {
        std::lock_guard<std::mutex> lock(hands_mutex_);
        q_left = q_abs;

        VectorXd q_right_send = q_right;
        VectorXd q_left_send  = q_left;

        q_right_send.head(4) = q_right_send.head(4) - q_right_baseline_.head(4);
        q_left_send.head(4)  = q_left_send.head(4)  - q_left_baseline_.head(4);

        // head_follow_hand(LEFT, q_left_send);
        Vector3d head_angles(h_roll, -h_pitch, -h_yaw);
        sendHandMotorCommands(q_right_send, q_left_send, head_angles);
        publish_trigger_pub_.publish(std_msgs::Empty());
    };

    std::thread right_thread;
    std::thread left_thread;

    if (req.right_enable) {
        right_thread = std::thread([&, right_cmds]()
        {
            using Eigen::Vector3d;
            using Eigen::Matrix3d;

            for (const std::string& raw : right_cmds) {
                if (user_exit.load()) break;

                std::string line = raw;
                trim(line);
                if (line.empty()) continue;
                if (line.size() == 1 && (line[0] == 'x' || line[0] == 'X')) {
                    user_exit.store(true);
                    break;
                }

                std::istringstream iss(line);
                std::string mode;
                if (!(iss >> mode)) continue;

                std::vector<double> vals;
                double tmp;
                while (iss >> tmp) vals.push_back(tmp);

                bool is_abs = (mode == "abs" || mode == "ABS");

                double mx = 0, my = 0, mz = 0;
                double gx = 0, gy = 0, gz = 0;
                double rx = 0, ry = 0, rz = 0;
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

                Matrix3d R0;
                Vector3d r0;
                {
                    std::lock_guard<std::mutex> lock(hands_mutex_);
                    hand_func_R.HO_FK_palm(q_right);
                    r0 = hand_func_R.r_palm;
                    R0 = hand_func_R.R_palm.block<3,3>(0,0);
                }

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
                    mid    = have_mid ? Vector3d(mx, my, mz) : 0.5 * (r0 + r_goal);
                    R_goal = R_inc;
                } else {
                    r_goal = r0 + Vector3d(gx, gy, gz);
                    mid    = 0.5 * (r0 + r_goal);
                    R_goal = R0 * R_inc;
                }

                if (!approachViaOneMid(hand_func_R, coef_generator, q_right,
                                       mid, r_goal, R_goal, T1, T2, T, pub_right)) {
                    right_ok.store(false);
                    break;
                }
            }
        });
    }

    if (req.left_enable) {
        left_thread = std::thread([&, left_cmds]()
        {
            using Eigen::Vector3d;
            using Eigen::Matrix3d;

            for (const std::string& raw : left_cmds) {
                if (user_exit.load()) break;

                std::string line = raw;
                trim(line);
                if (line.empty()) continue;
                if (line.size() == 1 && (line[0] == 'x' || line[0] == 'X')) {
                    user_exit.store(true);
                    break;
                }

                std::istringstream iss(line);
                std::string mode;
                if (!(iss >> mode)) continue;

                std::vector<double> vals;
                double tmp;
                while (iss >> tmp) vals.push_back(tmp);

                bool is_abs = (mode == "abs" || mode == "ABS");

                double mx = 0, my = 0, mz = 0;
                double gx = 0, gy = 0, gz = 0;
                double rx = 0, ry = 0, rz = 0;
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

                Matrix3d R0;
                Vector3d r0;
                {
                    std::lock_guard<std::mutex> lock(hands_mutex_);
                    hand_func_L.HO_FK_palm(q_left);
                    r0 = hand_func_L.r_palm;
                    R0 = hand_func_L.R_palm.block<3,3>(0,0);
                }

                const double RX = rx * M_PI/180.0;
                const double RY = ry * M_PI/180.0;
                const double RZ = rz * M_PI/180.0;

                Matrix3d R_inc = hand_func_L.rot(2, RY, 3)
                               * hand_func_L.rot(1, RX, 3)
                               * hand_func_L.rot(3, RZ, 3);

                Vector3d r_goal, mid;
                Matrix3d R_goal;

                if (is_abs) {
                    r_goal = Vector3d(gx, gy, gz);
                    mid    = have_mid ? Vector3d(mx, my, mz) : 0.5 * (r0 + r_goal);
                    R_goal = R_inc;
                } else {
                    r_goal = r0 + Vector3d(gx, gy, gz);
                    mid    = 0.5 * (r0 + r_goal);
                    R_goal = R0 * R_inc;
                }

                if (!approachViaOneMid(hand_func_L, coef_generator, q_left,
                                       mid, r_goal, R_goal, T1, T2, T, pub_left)) {
                    left_ok.store(false);
                    break;
                }
            }
        });
    }

    if (req.right_enable && right_thread.joinable()) {
        right_thread.join();
    }
    if (req.left_enable && left_thread.joinable()) {
        left_thread.join();
    }

    q_right_state_ = q_right;
    q_left_state_  = q_left;

    bool ok = true;
    if (req.right_enable && !right_ok.load()) ok = false;
    if (req.left_enable && !left_ok.load())   ok = false;

    res.ok = ok;
    if (!ok) {
        if (!right_ok.load() && !left_ok.load())
            res.message = "right and left approach failed";
        else if (!right_ok.load())
            res.message = "right approach failed";
        else
            res.message = "left approach failed";
    } else {
        res.message = user_exit.load() ? "exit" : "done";
    }

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
    double segment_duration = 2;
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
        sendHandMotorCommands(q_send_r, q_send_l, Eigen::Vector3d(0,0,0));
        publish_trigger_pub_.publish(std_msgs::Empty());
    };

    // Move motor 3 (index 2) back to home first 
    {
        double start_r = q_work_r(2);
        double end_r   = q_target_r(2);
        double start_l = q_work_l(2);
        double end_l   = q_target_l(2);

        for (int step = 0; step <= (M_segment-1); ++step) {
            double a = (M_segment > 0) ? static_cast<double>(step)/(M_segment-1) : 1.0;
            q_work_r(2) = start_r + (end_r - start_r)*a;
            q_work_l(2) = start_l + (end_l - start_l)*a;
            pub(q_work_r,q_work_l);
            ros::spinOnce();
            rate_.sleep();
        }
    }

    // Move motors 0–3 together to home
    {
        Eigen::VectorXd start_r = q_work_r.head(5);
        Eigen::VectorXd end_r   = q_target_r.head(5);
        Eigen::VectorXd start_l = q_work_l.head(5);
        Eigen::VectorXd end_l   = q_target_l.head(5);

        for (int step = 0; step <= M_segment; ++step) {
            double a = (M_segment > 0) ? static_cast<double>(step)/M_segment : 1.0;
            for (int j = 0; j < 5; ++j) {
                if (j == 2) continue; // skip motor 2
                q_work_r(j) = start_r(j) + (end_r(j) - start_r(j)) * a;
                q_work_l(j) = start_l(j) + (end_l(j) - start_l(j)) * a;
            }
            pub(q_work_r, q_work_l);
            ros::spinOnce();
            rate_.sleep();
        }
    }

    q_right_state_ = q_work_r;
    q_left_state_  = q_work_l;
    res.success = true;
    return true;
}

bool HandManager::arm_home_service_handler(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ros::Rate rate_(rate);
    
    // Reset hall sensor states
    for (int i = 0; i < 8; i++) {
        hall_sensors_state[i] = 0;
    }
    
    // Define arm joint mappings to hall sensors
    const int RIGHT_ELBOW_SENSOR = 0;
    const int RIGHT_YAW_SENSOR = 1;
    const int RIGHT_ROLL_SENSOR = 2;
    const int RIGHT_PITCH_SENSOR = 3;
    const int LEFT_ELBOW_SENSOR = 4;
    const int LEFT_YAW_SENSOR = 5;
    const int LEFT_ROLL_SENSOR = 6;
    const int LEFT_PITCH_SENSOR = 7;
    
    // Speed parameters
    const double HOMING_SPEED = 0.1;  // rad/s
    const double TIME_STEP = T;        // 0.005s 
    const double ROLL_OPEN_ANGLE = 10.0 * M_PI / 180;

    // Joint offsets for homing at desired angles in radians
    const double RIGHT_PITCH_OFFSET = 25.0 * M_PI / 180;
    const double RIGHT_ROLL_OFFSET = 2.0 * M_PI / 180;
    const double RIGHT_YAW_OFFSET = 93.0 * M_PI / 180;
    const double RIGHT_ELBOW_OFFSET = 12.0 * M_PI / 180;
    const double LEFT_PITCH_OFFSET = 33.0 * M_PI / 180;
    const double LEFT_ROLL_OFFSET = 9.0 * M_PI / 180;
    const double LEFT_YAW_OFFSET = 103.0 * M_PI / 180;
    const double LEFT_ELBOW_OFFSET = 21.0 * M_PI / 180;
    
    // Initialize joint positions (default to zero)
    Eigen::VectorXd q_right_current(7);
    Eigen::VectorXd q_left_current(7);
    q_right_current.setZero();
    q_left_current.setZero();
    
    // Initialize baseline positions to zero - will be set after homing
    q_right_baseline_.resize(7);
    q_left_baseline_.resize(7);
    q_right_baseline_.setZero();
    q_left_baseline_.setZero();
    
    // Homing sequence states
    enum HomingState {
        ROLL_OPENING,        // Step 1: Open roll joints for safety
        ELBOW_HOMING,        // Step 2: Home elbow joints using hall sensors
        YAW_HOMING,          // Step 3: Home yaw joints using hall sensors
        PITCH_HOMING,        // Step 4: Home pitch joints using hall sensors
        ROLL_HOMING,         // Step 5: Home roll joints using hall sensors
        ADD_DESIRED_ANGLES,  // Step 6: Add desired angles to each joint to put each joint in desired position
        COMPLETED            // All joints homed
    };
    
    HomingState right_state = ROLL_OPENING;
    HomingState left_state = ROLL_OPENING;
    
    // Boolean parameters
    bool right_roll_opened = false;
    bool left_roll_opened = false;
    bool is_right_joints_in_desired_home_pose = false;
    bool is_left_joints_in_desired_home_pose = false;
    
    // Joint direction multipliers
    const double RIGHT_PITCH_DIR = -1;  // negative for moving forward
    const double RIGHT_ROLL_DIR = -1;   // negative for moving away from robot's trunk
    const double RIGHT_YAW_DIR = 1;     // positive for moving through the body
    const double RIGHT_ELBOW_DIR = -1;  // negative for flexion
    
    const double LEFT_PITCH_DIR = -1;   // negative for moving forward
    const double LEFT_ROLL_DIR = 1;     // positive for moving away from robot's trunk
    const double LEFT_YAW_DIR = -1;     // negative for moving through the body
    const double LEFT_ELBOW_DIR = -1;   // negative for flexion
    
    int timeout_counter = 0;
    int right_arm_homing_offset_counter = 0;
    int left_arm_homing_offset_counter = 0;
    
    ROS_INFO("Starting homing sequence for both arms...");
    
    while ((right_state != COMPLETED || left_state != COMPLETED) && ros::ok()) {
        // cout << hall_sensors_state[0] << hall_sensors_state[1] << hall_sensors_state[2] << hall_sensors_state[3] << hall_sensors_state[4] << hall_sensors_state[5] << hall_sensors_state[6] << hall_sensors_state[7] << endl;
        if (timeout_counter > 100) {
            // Right arm homing sequence
            if (right_state == ROLL_OPENING) {
                if (!right_roll_opened) {
                    q_right_current(1) += HOMING_SPEED * TIME_STEP * RIGHT_ROLL_DIR; // arm_roll joint
                    if ((q_right_current(1)*RIGHT_ROLL_DIR) >= ROLL_OPEN_ANGLE) right_roll_opened = true;
                } else {
                    ROS_INFO("Right arm: Opening roll joint by 0.1 rad");
                    right_state = ELBOW_HOMING;
                }
            }
            else if (right_state == ELBOW_HOMING) {
                if (hall_sensors_state[RIGHT_ELBOW_SENSOR] == 0) {
                    double joint_speed = HOMING_SPEED * TIME_STEP * (-RIGHT_ELBOW_DIR);
                    q_right_current(3) += joint_speed; // elbow joint
                } else {
                    ROS_INFO("Right arm: Elbow joint homed");
                    right_state = YAW_HOMING;
                }
            }
            else if (right_state == YAW_HOMING) {
                if (hall_sensors_state[RIGHT_YAW_SENSOR] == 0) {
                    double joint_speed = HOMING_SPEED * TIME_STEP * (-RIGHT_YAW_DIR);
                    q_right_current(2) += joint_speed; // arm_yaw joint
                } else {
                    ROS_INFO("Right arm: Yaw joint homed");
                    right_state = PITCH_HOMING;
                }
            }
            else if (right_state == PITCH_HOMING) {
                if (hall_sensors_state[RIGHT_PITCH_SENSOR] == 0) {
                    double joint_speed = HOMING_SPEED * TIME_STEP * (-RIGHT_PITCH_DIR);
                    q_right_current(0) += joint_speed; // arm_pitch joint
                } else {
                    ROS_INFO("Right arm: Pitch joint homed");
                    right_state = ROLL_HOMING;
                }
            }
            else if (right_state == ROLL_HOMING) {
                if (hall_sensors_state[RIGHT_ROLL_SENSOR] == 0) {
                    double joint_speed = 0.5 * HOMING_SPEED * TIME_STEP * (-RIGHT_ROLL_DIR);
                    q_right_current(1) += joint_speed; // arm_roll joint
                } else {
                    ROS_INFO("Right arm: Roll joint homed");
                    right_state = ADD_DESIRED_ANGLES;
                }
            }
            else if (right_state == ADD_DESIRED_ANGLES) {
                if (!is_right_joints_in_desired_home_pose) {

                    if ((right_arm_homing_offset_counter * HOMING_SPEED * TIME_STEP) <= RIGHT_PITCH_OFFSET) {
                        q_right_current(0) += HOMING_SPEED * TIME_STEP * RIGHT_PITCH_DIR;}
                    if ((right_arm_homing_offset_counter * HOMING_SPEED * TIME_STEP) <= RIGHT_ROLL_OFFSET) {
                        q_right_current(1) += HOMING_SPEED * TIME_STEP * (-RIGHT_ROLL_DIR);}
                    if ((right_arm_homing_offset_counter * HOMING_SPEED * TIME_STEP) <= RIGHT_YAW_OFFSET) {
                        q_right_current(2) += HOMING_SPEED * TIME_STEP * RIGHT_YAW_DIR;}
                    if ((right_arm_homing_offset_counter * HOMING_SPEED * TIME_STEP) <= RIGHT_ELBOW_OFFSET) {
                        q_right_current(3) += HOMING_SPEED * TIME_STEP * RIGHT_ELBOW_DIR;}

                    if ( (right_arm_homing_offset_counter * HOMING_SPEED * TIME_STEP) >= std::max({RIGHT_PITCH_OFFSET,RIGHT_ROLL_OFFSET,RIGHT_YAW_OFFSET,RIGHT_ELBOW_OFFSET}) ) {
                        is_right_joints_in_desired_home_pose = true;}

                    right_arm_homing_offset_counter++;

                } else {
                    ROS_INFO("Right Arm in Home State");
                    right_state = COMPLETED;
                }
            }

            // Left arm homing sequence
            if (left_state == ROLL_OPENING) {
                if (!left_roll_opened) {
                    q_left_current(1) += HOMING_SPEED * TIME_STEP * LEFT_ROLL_DIR; // arm_roll joint
                    if ((q_left_current(1)*LEFT_ROLL_DIR) >= ROLL_OPEN_ANGLE) left_roll_opened = true;
                } else {
                    ROS_INFO("Left arm: Opening roll joint by 0.1 rad");
                    left_state = ELBOW_HOMING;
                }
            }
            else if (left_state == ELBOW_HOMING) {
                if (hall_sensors_state[LEFT_ELBOW_SENSOR] == 0) {
                    double joint_speed = HOMING_SPEED * TIME_STEP * (-LEFT_ELBOW_DIR);
                    q_left_current(3) += joint_speed; // elbow joint
                } else {
                    ROS_INFO("Left arm: Elbow joint homed");
                    left_state = YAW_HOMING;
                }
            }
            else if (left_state == YAW_HOMING) {
                if (hall_sensors_state[LEFT_YAW_SENSOR] == 0) {
                    double joint_speed = HOMING_SPEED * TIME_STEP * (-LEFT_YAW_DIR);
                    q_left_current(2) += joint_speed; // arm_yaw joint
                } else {
                    ROS_INFO("Left arm: Yaw joint homed");
                    left_state = PITCH_HOMING;
                }
            }
            else if (left_state == PITCH_HOMING) {
                if (hall_sensors_state[LEFT_PITCH_SENSOR] == 0) {
                    double joint_speed = HOMING_SPEED * TIME_STEP * (-LEFT_PITCH_DIR);
                    q_left_current(0) += joint_speed; // arm_pitch joint
                } else {
                    ROS_INFO("Left arm: Pitch joint homed");
                    left_state = ROLL_HOMING;
                }
            }
            else if (left_state == ROLL_HOMING) {
                if (hall_sensors_state[LEFT_ROLL_SENSOR] == 0) {
                    double joint_speed = 0.5 * HOMING_SPEED * TIME_STEP * (-LEFT_ROLL_DIR);
                    q_left_current(1) += joint_speed; // arm_roll joint
                } else {
                    ROS_INFO("Left arm: Roll joint homed");
                    left_state = ADD_DESIRED_ANGLES;
                }
            }
            else if (left_state == ADD_DESIRED_ANGLES) {
                if (!is_left_joints_in_desired_home_pose) {

                    if ((left_arm_homing_offset_counter * HOMING_SPEED * TIME_STEP) <= LEFT_PITCH_OFFSET) {
                        q_left_current(0) += HOMING_SPEED * TIME_STEP * LEFT_PITCH_DIR;}
                    if ((left_arm_homing_offset_counter * HOMING_SPEED * TIME_STEP) <= LEFT_ROLL_OFFSET) {
                        q_left_current(1) += HOMING_SPEED * TIME_STEP * (-LEFT_ROLL_DIR);}
                    if ((left_arm_homing_offset_counter * HOMING_SPEED * TIME_STEP) <= LEFT_YAW_OFFSET) {
                        q_left_current(2) += HOMING_SPEED * TIME_STEP * LEFT_YAW_DIR;}
                    if ((left_arm_homing_offset_counter * HOMING_SPEED * TIME_STEP) <= LEFT_ELBOW_OFFSET) {
                        q_left_current(3) += HOMING_SPEED * TIME_STEP * LEFT_ELBOW_DIR;}

                    if ( (left_arm_homing_offset_counter * HOMING_SPEED * TIME_STEP) >= std::max({LEFT_PITCH_OFFSET,LEFT_ROLL_OFFSET,LEFT_YAW_OFFSET,LEFT_ELBOW_OFFSET}) ) {
                        is_left_joints_in_desired_home_pose = true;}

                    left_arm_homing_offset_counter++;

                } else {
                    ROS_INFO("Left Arm in Home State");
                    left_state = COMPLETED;
                }
            }
        } else {
            for (int i = 0; i < 8; i++) {
                hall_sensors_state[i] = 0;
            }
        }
        
        // Publish motor commands
        Eigen::VectorXd q_send_right = q_right_current;
        q_send_right.head(4) = q_right_current.head(4) - q_right_baseline_.head(4);
        
        Eigen::VectorXd q_send_left = q_left_current;
        q_send_left.head(4) = q_left_current.head(4) - q_left_baseline_.head(4);

        sendHandMotorCommands(q_send_right, q_send_left, Eigen::Vector3d(0, 0, 0));
        publish_trigger_pub_.publish(std_msgs::Empty());
        rate_.sleep();
        timeout_counter++; 
    }
    
    if (right_state == COMPLETED && left_state == COMPLETED) {
        ROS_INFO("Arm homing completed successfully - all joints homed");
        
        // Store the final homed positions as the new baseline and state
        q_right_baseline_ = q_right_current;
        q_left_baseline_ = q_left_current;
        q_right_state_ = q_right_current;
        q_left_state_ = q_left_current;
        
        // Reset hall sensor states for next use
        for (int i = 0; i < 8; i++) {
            hall_sensors_state[i] = 0;
        }
        
        return true;
    } else {
        ROS_ERROR("Arm homing failed.");
        return false;
    }
}

bool HandManager::fingerControlService(hand_planner::FingerControl::Request &req, hand_planner::FingerControl::Response &res) {
    try {
        // Convert hand selection to enum (default to RIGHT_HAND if not specified)
        HandSelection hand = (req.hand_selection.empty()) ? HandSelection::RIGHT_HAND : finger_control_->stringToHandSelection(req.hand_selection);
        std::vector<uint8_t> target_positions(req.target_positions.begin(), req.target_positions.end());
        uint8_t control_data = req.control_data;

        bool success = finger_control_->setDirectControl(target_positions, control_data, hand);

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
    return true;
}

bool HandManager::fingerScenarioService(hand_planner::FingerScenario::Request &req, hand_planner::FingerScenario::Response &res) {  
    try {
        // Convert hand selection to enum (default to RIGHT_HAND if not specified)
        HandSelection hand = (req.hand_selection.empty()) ? HandSelection::RIGHT_HAND : finger_control_->stringToHandSelection(req.hand_selection);        
        
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