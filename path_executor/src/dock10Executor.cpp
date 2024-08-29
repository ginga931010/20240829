#include "dock10Executor.h"
Dock10Executor::Dock10Executor(ros::NodeHandle& nh, ros::NodeHandle& nh_local) {
    nh_ = nh;
    nh_local_ = nh_local;
    std_srvs::Empty empt;
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("params", &Dock10Executor::initializeParams, this);
    initializeParams(empt.request, empt.response);
    initialize();
}

Dock10Executor::~Dock10Executor() {
    nh_local_.deleteParam("active");
    nh_local_.deleteParam("control_frequency");
    nh_local_.deleteParam(robot_type_ + "/linear_max_velocity");
    nh_local_.deleteParam(robot_type_ + "/profile_percent");
    nh_local_.deleteParam(robot_type_ + "/stop_tolerance");
    nh_local_.deleteParam(robot_type_ + "/pose_type");
    nh_local_.deleteParam(robot_type_ + "/angular_kp");
    nh_local_.deleteParam(robot_type_ + "/fast_angular_max_vel");
    nh_local_.deleteParam(robot_type_ + "/fast_angular_kp");
    // nh_local_.deleteParam("rival_tolerance");
}

void Dock10Executor::initialize() {
    // zeroing the arrays
    goal_[0] = 0.0;
    goal_[1] = 0.0;
    goal_[2] = 0.0;

    pose_[0] = 0.0;
    pose_[1] = 0.0;
    pose_[2] = 0.0;

    vel_[0] = 0.0;
    vel_[1] = 0.0;
    vel_[2] = 0.0;

    dock_dist_ = 0.05;
    if_get_goal_ = false;
    count_dock_dist_ = false;
    // rival_dist_ = 10.0;
    dist_ = 0.0;
    vibrate_time_now_ = 0;
    // linear_max_vel_ = 0.0;
    // angular_max_vel_ = 0.0;

    mode_ = MODE::IDLE;

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &Dock10Executor::timerCB, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency_), false);
    timer_.start();

    rival_dist_ = 100;
    mini_dist_ = 100;

    debounce = 0;
    debounce_ang = 0;

    rival_x_ = 0;
    rival_y_ = 0;
}

bool Dock10Executor::initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    // Load parameter
    bool get_param_ok = true;
    bool prev_active = p_active_;
    get_param_ok = nh_local_.param<bool>("active", p_active_, true);
    // get_param_ok = nh_local_.param<string>("", _, "");
    get_param_ok &= nh_local_.param<double>("control_frequency", control_frequency_, 50);
    get_param_ok &= nh_local_.param<std::string>("robot_type", robot_type_, "holonomic");
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_max_velocity", linear_max_vel_, 0.3);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_kp", linear_kp_, 2.0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_kd", linear_kd_, 1.0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_acceleration", linear_acceleration_, 0.15);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/slow_linear_max_velocity", slow_linear_max_vel_, 0.7);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/slow_linear_kp", slow_linear_kp_, 3.5);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/slow_linear_acceleration", slow_acceleration_, 1.0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/linear_min_velocity", linear_min_vel_, 0.1); 

    get_param_ok &= nh_local_.param<double>(robot_type_ + "/angular_max_velocity", angular_max_vel_, 1);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/angular_min_velocity", angular_min_vel_, 0.3);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/angular_kp", angular_kp_, 2.5);
    get_param_ok &= nh_local_.param<double>("angular_velocity_divider", div_, 3.0);                               // Not in yaml
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/fast_angular_max_vel", fast_angular_max_vel_, 3.0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/fast_angular_kp", fast_angular_kp_, 2.0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/profile_percent", profile_percent_, 0.2);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/point_stop_tolerance", tolerance_, 0.005);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/angle_stop_tolerance", ang_tolerance_, 0.01);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/first_angle_stop_tolerance", first_ang_tolerance_, 0.01);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/move_fail_tolerance", move_fail_tolerance_, 1.05);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/move_over_tolerance", move_over_tolerance_, 0.02);
    get_param_ok &= nh_local_.param<double>("first_rotate_range", first_rotate_range_, 0.1);                     // Not in yaml
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/angular_adjust_kp", angular_adjust_kp_, 0.8);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/angular_adjust_limit", angular_adjust_limit_, 0.1);


    get_param_ok &= nh_local_.param<int>("pose_type", pose_type_, 0);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "/rival_tolerance", rival_tolerance_, 0.40);
    get_param_ok &= nh_local_.param<double>(robot_type_ + "rival_parallel_tolerance", rival_parallel_tolerance_, 0.4);

    if (p_active_ != prev_active) {
        if (p_active_) {
            goal_sub_ = nh_.subscribe("dock10_exec_goal", 50, &Dock10Executor::goalCB, this);
            if (pose_type_ == 0) {
                pose_sub_ = nh_.subscribe("odom", 50, &Dock10Executor::poseCB_Odometry, this);
            } else if (pose_type_ == 1) {
                pose_sub_ = nh_.subscribe("final_pose", 50, &Dock10Executor::poseCB_Odometry, this);
            }
            rival_sub_ = nh_.subscribe("/rival/final_pose", 50, &Dock10Executor::rivalCB_Odometry, this);
            // rival2_sub_ = nh_.subscribe("/rival2/odom", 50, &DockExecutor::rivalCB_Odometry, this);
            pub_ = nh_.advertise<geometry_msgs::Twist>("dock10_exec_cmd_vel", 1);
            goalreachedPub_ = nh_.advertise<std_msgs::Char>("dock_exec_status", 1);
            fast_mode_client = nh_.serviceClient<std_srvs::SetBool>("fast_spin");
        } else {
            goal_sub_.shutdown();
            pose_sub_.shutdown();
            pub_.shutdown();
        }
    }

    if (get_param_ok) {
        ROS_INFO_STREAM("[Docking10 Executor]: "
                        << "Set params ok");
    } else {
        ROS_WARN_STREAM("[Docking10 Executor]: "
                        << "Set params failed");
    }
    return true;
}

double Dock10Executor::distance(double x1, double y1, double x2, double y2) {
    double distance = 0.0;
    distance = hypot((x2 - x1), (y2 - y1));
    return distance;
}

void Dock10Executor::velocityPUB() {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vel_[0];
    cmd_vel.linear.y = vel_[1];
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = vel_[2];
    pub_.publish(cmd_vel);
}

void Dock10Executor::timerCB(const ros::TimerEvent& e) {
    if (if_get_goal_) {

        dist_ = distance(pose_[0], pose_[1], goal_[0], goal_[1]);
        ang_diff_ = goal_[2] - pose_[2];
        first_ang_diff_ = atan2((goal_[1] - pose_[1]),(goal_[0] - pose_[0])) - pose_[2];
        /*if_drive_straight_ = 1;
        if (first_ang_diff_ > M_PI * 3.0 / 2.0) first_ang_diff_ = first_ang_diff_ - 2 * M_PI;
        else if (first_ang_diff_ < - M_PI * 3.0 / 2.0) first_ang_diff_ = first_ang_diff_ + 2 * M_PI;

        if (first_ang_diff_ > M_PI / 2.0) {
            if_drive_straight_ = -1;
            first_ang_diff_ = first_ang_diff_ - M_PI;
        }
        else if (first_ang_diff_ < -M_PI / 2.0) {
            if_drive_straight_ = -1;
            first_ang_diff_ = M_PI + first_ang_diff_;
        }

        if (last_drive_straight_ == 0) last_drive_straight_ = if_drive_straight_;
        
        if(dist_ < mini_dist_){
            mini_dist_ = dist_;
        }*/


        t_now_ = ros::Time::now().toSec();
        dt_ = t_now_ - t_bef_;

        switch (mode_) {
            case MODE::MOVE: {
                // ROS_INFO_STREAM("start move");
                move();
                break;
            }
            case MODE::ROTATE: {
                rotate();
                break;
            }
            case MODE::IDLE: {
                break;
            }
        }
        // publish cmd_vel
        velocityPUB();
    }

    // ROS_INFO("%f %f %f", vel_[0], vel_[1], dt_);

    // remember the time when leaving this loop
    t_bef_ = ros::Time::now().toSec();
}
void Dock10Executor::move() {
    if(distance_ <= tolerance_){
        vel_[0] = vel_[1] = vel_[2] = 0.0;
        velocityPUB();
        mode_ = MODE::ROTATE;
    }
    else{
        vel_[0] = (goal_[0] - pose_[0]) * cur_linear_kp_;
        vel_[1] = (goal_[1] - pose_[1]) * cur_linear_kp_;
        vel_[2] = 0;
        velocityPUB();
    }
    
}

void Dock10Executor::rotate() {
    ang_diff_ = goal_[2] - pose_[2];

    while (ang_diff_ >= M_PI) {
        ang_diff_ -= 2 * M_PI;
    }
    while (ang_diff_ <= -M_PI) {
        ang_diff_ += 2 * M_PI;
    }
    double ang_vel = std::min(angular_max_vel_, fabs(ang_diff_) * angular_kp_);
    // if (ang_vel < angular_min_vel_) ang_vel = angular_min_vel_;
    // ROS_INFO("%f, %f", ang_diff_, ang_tolerance_);
    if (fabs(ang_diff_) < ang_tolerance_) {
        debounce_ang ++;
    } else {
        debounce_ang = 0;
    }

    if(debounce_ang >= 6){
        vel_[0] = vel_[1] = vel_[2] = 0.0;
        velocityPUB();
        ROS_INFO("[Dock Executor] : Successfully dock-rotated!");
        if_get_goal_ = false;
        std_msgs::Char finished;
        finished.data = 1;
        goalreachedPub_.publish(finished);
        ROS_INFO("[Dock Executor] : Successfully dock-arrive-goal!");
    } else {
        // ROS_INFO("[Dock Executor] : debounce_ang -> %d", debounce_ang);
    }

    if (ang_diff_ > 0)
        ang_vel = ang_vel;
    else if (ang_diff_ < 0)
        ang_vel = -1 * ang_vel;

    // ROS_INFO("ang_diff_: %f",ang_diff_);
    // ROS_INFO("ang_vel: %f",ang_vel);
    vel_[0] = vel_[1] = 0.0;
    vel_[2] = ang_vel;
    velocityPUB();
}
void Dock10Executor::goalCB(const geometry_msgs::PoseStamped& data) {
    ROS_INFO_STREAM("[Dock Executor]: In the goalCB!");

    vel_[0] = vel_[1] = vel_[2] = 0.0;
    if (data.pose.position.x == -1 && data.pose.position.y == -1) {
        ROS_INFO("[Dock Executor]: Mission Abort!");
        mode_ = MODE::IDLE;
        return;
    }
    // ROS_INFO("[Dock Executor]: Dock goal received! (%f, %f)", data.pose.position.x, data.pose.position.y);

    tf2::Quaternion q;
    tf2::fromMsg(data.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    t_now_ = ros::Time::now().toSec();

    if (data.header.frame_id == "dock") {
        cur_linear_max_vel_ = linear_max_vel_;
        cur_linear_acceleration_ = linear_acceleration_;
        cur_linear_kp_ = linear_kp_;
    }
    else if (data.header.frame_id == "dock10") {
        cur_linear_max_vel_ = linear_max_vel_;
        cur_linear_acceleration_ = linear_acceleration_;
        cur_linear_kp_ = linear_kp_;
    }
    else if (data.header.frame_id == "slow-dock") {
        cur_linear_max_vel_ = slow_linear_max_vel_;
        cur_linear_acceleration_ = slow_acceleration_;
        cur_linear_kp_ = slow_linear_kp_;
    }
    else {
        ROS_WARN_STREAM("No such dock mode: " << data.header.frame_id);
        cur_linear_max_vel_ = linear_max_vel_;
        cur_linear_acceleration_ = linear_acceleration_;
        cur_linear_kp_ = linear_kp_;
    }

    goal_[0] = data.pose.position.x;  // + dock_dist_*cos(yaw);
    goal_[1] = data.pose.position.y;  // + dock_dist_*sin(yaw);
    goal_[2] = yaw;


    dist_ = distance(pose_[0], pose_[1], goal_[0], goal_[1]);
    ang_diff_ = goal_[2] - pose_[2];
    first_ang_diff_ = atan2((goal_[1] - pose_[1]),(goal_[0] - pose_[0])) - pose_[2];

    // ROS_INFO(" pose : %f %f %f vel : %f %f %f", pose_  [0],pose_[1],pose_[2], vel_[0],vel_[1],vel_[2]);
    // ROS_INFO(" dis : %f ,ang_dif :%f ,first_angle_diff: %f ", dist_,ang_diff_,first_ang_diff_);
    
    
    // ROS_INFO(" dis : %f ,ang_dif :%f ,first_angle_diff: %f ", dist_,ang_diff_,first_ang_diff_);

    first_rot_need_time_ = fabs(first_ang_diff_ / angular_max_vel_);

    t_first_rot_ = t_now_;

    ROS_INFO_STREAM(t_now_);
    mode_ = MODE::MOVE;
    ROS_INFO("[Dock Executor]: Set Mode to DOCK FIRST ROTATE!");
    // ROS_INFO("[Dock Executor]: Set Mode to DOCK FIRST ROTATE!");

    mini_dist_ = 100;
    if_get_goal_ = true;
    dacc_start_ = false;
    debounce_ang = 0;
    t_bef_ = ros::Time::now().toSec();
}

void Dock10Executor::poseCB_Odometry(const nav_msgs::Odometry& data) {
    pose_[0] = data.pose.pose.position.x;
    pose_[1] = data.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(data.pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    pose_[2] = yaw;

    ROS_INFO("igetthepose");
}

void Dock10Executor::poseCB_PoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped& data) {
    pose_[0] = data.pose.pose.position.x;
    pose_[1] = data.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(data.pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    pose_[2] = yaw;
    //** ROS_INFO("odom: %f %f", pose_[0], pose_[1]);
}

void Dock10Executor::rivalCB_Odometry(const nav_msgs::Odometry& data){
    double rival_x_ = data.pose.pose.position.x;
    double rival_y_ = data.pose.pose.position.y;
    if (rival_x_ < 0 || rival_x_ > 3.0 || rival_y_ < 0 || rival_y_ > 2.0) {
        rival_dist_ = 100;
        return ;
    }
    double dist = distance(pose_[0], pose_[1], data.pose.pose.position.x, data.pose.pose.position.y);
    rival_dist_ =  dist;
    // ROS_INFO("[Dock Executor]: Rival distance: %f",rival_dist_);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dock10Executor");
    ros::NodeHandle nh(""), nh_local("~");
    Dock10Executor dock10Executor(nh, nh_local);
    ROS_WARN("this node is running");
    while (ros::ok()) {
        ros::spin();
    }
}