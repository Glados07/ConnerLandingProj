#include <ros/ros.h>

#include <std_msgs/Bool.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

// #include <ros_demo_1/misc_param.h>

class idle_def_process
{
private:
    ros::NodeHandle nh;
    ros::Rate rate;

    // Subscribers
    mavros_msgs::State curr_state;
    ros::Subscriber state_sub;

    bool flying_flag;
    ros::Subscriber flying_flag_sub;

    // Publishers
    mavros_msgs::AttitudeTarget att_motion_cmd;
    ros::Publisher att_motion_cmd_pub;

public:
    idle_def_process(float rate_hz) : rate(rate_hz),
                                      flying_flag(false)
    {
        // Subscribers
        state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &idle_def_process::state_cb, this);
        flying_flag_sub = nh.subscribe<std_msgs::Bool>("flying_flag", 10, &idle_def_process::flying_flag_cb, this);

        // Publishers
        att_motion_cmd_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    }

    void run()
    {
        wait_for_arm();
        main_loop();
    }

private:
    // Callbacks
    void state_cb(const mavros_msgs::State::ConstPtr &msg)
    {
        curr_state = *msg;
    }

    void flying_flag_cb(const std_msgs::BoolConstPtr &msg)
    {
        bool prev_flying = flying_flag;
        flying_flag = msg->data;

        if (prev_flying && !flying_flag)
        {
            // 飞行结束，允许发送新的plan请求
            ROS_INFO("Planner finished flying. Ready for next plan.");
        }
        else if (!prev_flying && flying_flag)
        {
            // 飞行开始
            ROS_INFO("Planner started flying. Offb_node yielding control.");
        }
    }

    void wait_for_arm()
    {
        ROS_INFO("Waiting for vehicle to be armed...");
        ros::Time start_time = ros::Time::now();
        while (ros::ok())
        {
            if (curr_state.armed && curr_state.mode == "OFFBOARD")
            {
                return;
            }
            // Overtime protection
            if ((ros::Time::now() - start_time) > ros::Duration(60.0))
            {
                ROS_WARN("Idle Node: Wait for arm timeout. Exiting...");
                ros::shutdown();
                return;
            }
            rate.sleep();
        }
    }

    void main_loop()
    {
        ROS_INFO("Entering main control loop (default idle)...");
        while (ros::ok())
        {
            if (!flying_flag)
            {
                ROS_INFO_THROTTLE(5, "Default Idle Enabled.");
                cmd_once_default_idle();
            }
            else
            {
                ROS_INFO_THROTTLE(1, "Currently flying. Quit from default idle.");
            }
            rate.sleep();
        }
    }

    void cmd_once_default_idle()
    {
        att_motion_cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        att_motion_cmd.body_rate.x = 0;
        att_motion_cmd.body_rate.y = 0;
        att_motion_cmd.body_rate.z = 0;
        att_motion_cmd.thrust = 0.0; // MISCParam::idling_thrust;
        att_motion_cmd.header.stamp = ros::Time::now();
        att_motion_cmd_pub.publish(att_motion_cmd);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "idle_def_node");
    ros::NodeHandle nh;

    idle_def_process controller(50.0); // 50Hz control loop

    // Multithreading for callbacks
    ros::AsyncSpinner spinner(2); // Use 2 threads for callbacks
    spinner.start();

    controller.run();

    return 0;
}