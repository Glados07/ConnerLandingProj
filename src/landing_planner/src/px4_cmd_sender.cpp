#include <ros/ros.h>
#include <mavros_msgs/State.h>

#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <landing_planner/PolyTraj.h>
#include <landing_planner/Trajectory.h>

class PX4CmdSender {
private:
    ros::NodeHandle nh;
    ros::Rate rate;

    // Subscriber
    PolyTraj::Traj_7od x_coeffs, y_coeffs, z_coeffs;
    PolyTraj::Traj_3od yaw_coeffs;
    double traj_time_left;
    ros::Subscriber planner_traj_sub;

    bool is_taking_off;
    ros::Subscriber tkof_cmd_sub;

    mavros_msgs::State curr_state;
    ros::Subscriber state_sub;

    // Publisher
    mavros_msgs::PositionTarget pos_target;
    ros::Publisher pos_target_pub;

    mavros_msgs::AttitudeTarget att_target;
    ros::Publisher att_target_pub;

    std_msgs::Bool flying_flag;
    ros::Publisher flying_flag_pub;

    ros::Time exec_start_time;
    ros::Time Timer;
public:
    PX4CmdSender(float rate_hz) : rate(rate_hz), traj_time_left(0.0), is_taking_off(false){
        // Subscriber
        state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &PX4CmdSender::state_cb, this);
        planner_traj_sub = nh.subscribe<landing_planner::Trajectory>("planned_trajectory", 10, &PX4CmdSender::exec_traj_cb, this);
        tkof_cmd_sub = nh.subscribe<std_msgs::Bool>("tkof_cmd", 10, &PX4CmdSender::tkof_cmd_cb, this);

        // Publisher
        pos_target_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
        att_target_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
        flying_flag_pub = nh.advertise<std_msgs::Bool>("flying_flag", 10);
    }
    void run(){
        wait_for_arm();
        main_loop();
    }
private:
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        curr_state = *msg;
    }
    
    void tkof_cmd_cb(const std_msgs::Bool::ConstPtr& msg){
        is_taking_off = msg->data;
        
        if(is_taking_off) {
            // Send takeoff thrust command
            const double TAKEOFF_IMPULSE_THRUST = 0.8;  // normalized thrust (0-1)
            
            att_target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
            att_target.body_rate.x = 0;
            att_target.body_rate.y = 0;
            att_target.body_rate.z = 0;
            att_target.thrust = TAKEOFF_IMPULSE_THRUST;
            att_target.header.stamp = ros::Time::now();
            att_target_pub.publish(att_target);
            
            ROS_INFO_THROTTLE(0.5, "Cmder: Executing takeoff with thrust=%.2f", TAKEOFF_IMPULSE_THRUST);
        }
    }
    
    void exec_traj_cb(const landing_planner::Trajectory::ConstPtr& msg){
        // Extract trajectory coefficients and duration
        ROS_INFO("Cmder: Received new trajectory from planner.");
        std::copy(msg->x_coeffs.begin(), msg->x_coeffs.end(), x_coeffs.begin());
        std::copy(msg->y_coeffs.begin(), msg->y_coeffs.end(), y_coeffs.begin());
        std::copy(msg->z_coeffs.begin(), msg->z_coeffs.end(), z_coeffs.begin());
        std::copy(msg->yaw_coeffs.begin(), msg->yaw_coeffs.end(), yaw_coeffs.begin());
        traj_time_left = msg->duration;
        exec_start_time = ros::Time::now();

    }
    void wait_for_arm() {
        ROS_INFO("Cmder Node: Waiting for vehicle to be armed...");
        ros::Time start_time = ros::Time::now();
        while (ros::ok()){
            if (curr_state.armed && curr_state.mode == "OFFBOARD"){
                return;
            }
            // Overtime protection
            if ((ros::Time::now() - start_time) > ros::Duration(60.0)){
                ROS_WARN("Cmder: Wait for arm timeout after 60 seconds. Exiting...");
                ros::shutdown();
                return;
            }
            rate.sleep();
        }
    }

    void main_loop() {
        ROS_INFO("Cmder: Vehicle armed. Starting main loop.");
        while (ros::ok()){
            Timer = ros::Time::now();
            
            // Priority 1: Handle takeoff
            if(is_taking_off) {
                flying_flag.data = true;
                flying_flag_pub.publish(flying_flag);
                // Takeoff command is sent in tkof_cmd_cb
            }
            // Priority 2: Execute trajectory (only if not taking off)
            else if(Timer - exec_start_time < ros::Duration(traj_time_left)){
                flying_flag.data = true;
                flying_flag_pub.publish(flying_flag);
                
                // Execute trajectory for one step
                double t = (Timer - exec_start_time).toSec();
                
                // Position control
                pos_target.header.stamp = ros::Time::now();
                pos_target.header.frame_id = "map";
                pos_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                pos_target.type_mask = 0; 
                
                pos_target.position.x = PolyTraj::PosX_AtTime_7od(x_coeffs, t);
                pos_target.position.y = PolyTraj::PosX_AtTime_7od(y_coeffs, t);
                pos_target.position.z = PolyTraj::PosX_AtTime_7od(z_coeffs, t);
                pos_target.velocity.x = PolyTraj::VelX_AtTime_7od(x_coeffs, t);
                pos_target.velocity.y = PolyTraj::VelX_AtTime_7od(y_coeffs, t);
                pos_target.velocity.z = PolyTraj::VelX_AtTime_7od(z_coeffs, t);
                pos_target.acceleration_or_force.x = PolyTraj::AccX_AtTime_7od(x_coeffs, t);
                pos_target.acceleration_or_force.y = PolyTraj::AccX_AtTime_7od(y_coeffs, t);
                pos_target.acceleration_or_force.z = PolyTraj::AccX_AtTime_7od(z_coeffs, t);
                pos_target.yaw = PolyTraj::PosX_AtTime_3od(yaw_coeffs, t);
                pos_target.yaw_rate = PolyTraj::VelX_AtTime_3od(yaw_coeffs, t);
                
                pos_target_pub.publish(pos_target);
            } 
            // Priority 3: Landed/idle
            else {
                flying_flag.data = false;
                flying_flag_pub.publish(flying_flag);
            }
            rate.sleep();
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "px4_cmd_sender_node");
    ros::NodeHandle nh;

    PX4CmdSender cmd_sender(50.0); // 50 Hz

    // Multithreading for callbacks
    ros::AsyncSpinner spinner(2); // Use 2 threads for callbacks
    spinner.start();
    cmd_sender.run();
    return 0;
}