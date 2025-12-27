#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

class offb_arm_hd {
private:
    ros::NodeHandle nh;
    ros::Rate rate;
    // ROS communication objects
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    
    // Subscribers
    ros::Subscriber state_sub;
    mavros_msgs::State curr_state;
    
    // Publishers
    ros::Publisher att_motion_cmd_pub;
    mavros_msgs::AttitudeTarget att_motion_cmd;
    
    // Control parameters
    ros::Time last_request;
    
    // Timeout parameters
    static constexpr double CONNECTION_TIMEOUT = 30.0;  // 30 seconds for connection
    static constexpr double OFFBOARD_TIMEOUT = 30.0;    // 30 seconds for offboard mode
    static constexpr double ARMING_TIMEOUT = 60.0;      // 60 seconds total timeout

public:
    offb_arm_hd(float rate_hz) : 
        rate(rate_hz) {
        // Service clients
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        
        // Subscribers
        state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &offb_arm_hd::state_cb, this);
        
        // Publishers
        att_motion_cmd_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    }

    void run() {
        wait_for_connection();
        setmode_and_arming();
    }

private:
    // Callbacks
    void state_cb(const mavros_msgs::State::ConstPtr& msg) { 
        curr_state = *msg; 
    }

    void wait_for_connection() {
        ros::Time start_time = ros::Time::now();
        while(ros::ok() && !curr_state.connected) {
            if((ros::Time::now() - start_time).toSec() > CONNECTION_TIMEOUT) {
                ROS_ERROR("Connection timeout after %.1f seconds. FCU not connected.", CONNECTION_TIMEOUT);
                ros::shutdown();
                return;
            }
            rate.sleep();
        }
        ROS_INFO("FCU connected");
    }

    void setmode_and_arming() {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        last_request = ros::Time::now();
        ros::Time start_time = ros::Time::now();

        ROS_INFO("Starting arming and offboard mode setup...");
        while(ros::ok()) {
            // Check for total timeout
            double elapsed = (ros::Time::now() - start_time).toSec();
            if(elapsed > ARMING_TIMEOUT) {
                ROS_ERROR("Arming and offboard setup timeout after %.1f seconds.", ARMING_TIMEOUT);
                ROS_ERROR("Current state - Armed: %s, Mode: %s", 
                         curr_state.armed ? "true" : "false", 
                         curr_state.mode.c_str());
                ros::shutdown();
                return;
            }
            
            cmd_once_default_idle();
            offboard_enable(offb_set_mode); //如果当前是OFFBOARD模式，则该函数无效，不会更新last_request
            arming_enable(arm_cmd); //如果当前是armed，则该函数无效，不会更新last_request
            
            if(curr_state.armed && curr_state.mode == "OFFBOARD") {
                ROS_INFO("Vehicle armed and in OFFBOARD mode successfully (%.1f seconds)", elapsed);
                return; // Exit if armed and in OFFBOARD mode
            }
            rate.sleep();
        }
    }

    void offboard_enable(mavros_msgs::SetMode& offb_set_mode) {
        if(curr_state.mode != "OFFBOARD" && 
           (ros::Time::now() - last_request > ros::Duration(5.0))) 
        {
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
    }

    void arming_enable(mavros_msgs::CommandBool& arm_cmd) {
        if(curr_state.mode == "OFFBOARD" && 
           !curr_state.armed &&
           (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
    }

    void cmd_once_default_idle() {
        att_motion_cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        att_motion_cmd.body_rate.x = 0;
        att_motion_cmd.body_rate.y = 0;
        att_motion_cmd.body_rate.z = 0;
        att_motion_cmd.thrust = 0.1; // Minimal thrust to keep vehicle alive
        att_motion_cmd.header.stamp = ros::Time::now();
        att_motion_cmd_pub.publish(att_motion_cmd);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_arm_enable_node");
    ros::NodeHandle nh;

    offb_arm_hd offb_hder(50.0); // 50Hz control loop
    ROS_INFO("Offboard & arm enabling...");

    // Multithreading for callbacks
    ros::AsyncSpinner spinner(2); // Use 2 threads for callbacks
    spinner.start();

    offb_hder.run();
    ROS_INFO("Enabling node finished.");

    return 0;
}