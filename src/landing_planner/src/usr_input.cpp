#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <landing_planner/PlanRequest.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <limits>

class UserInputNode
{
private:
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber flying_flag_sub;
    ros::Subscriber state_sub;
    
    bool is_flying;
    bool previous_flying_state;
    
    mavros_msgs::State current_state;
    bool first_request_done;

    // Publishers
    ros::Publisher plan_request_pub;

public:
    UserInputNode() : is_flying(false), previous_flying_state(false), first_request_done(false)
    {
        // Subscribe to mavros state
        state_sub = nh.subscribe<mavros_msgs::State>(
            "mavros/state", 10, &UserInputNode::state_cb, this);
        
        // Subscribe to flying flag
        flying_flag_sub = nh.subscribe<std_msgs::Bool>(
            "flying_flag", 10, &UserInputNode::flying_flag_cb, this);

        // Publisher for plan request
        plan_request_pub = nh.advertise<landing_planner::PlanRequest>(
            "plan_request", 10);

        ROS_INFO("UI Node: Initialized. Waiting for vehicle to arm...");
    }

    void run()
    {
        ros::Rate rate(10.0); // 10Hz

        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // Callback for mavros state
    void state_cb(const mavros_msgs::State::ConstPtr &msg)
    {
        current_state = *msg;
        
        // First request: trigger when armed and in OFFBOARD mode
        if (!first_request_done && current_state.armed && current_state.mode == "OFFBOARD")
        {
            ROS_INFO("Vehicle armed and OFFBOARD mode detected. Ready for first user input.");
            first_request_done = true;
            
            // Request user input
            requestUserInput();
        }
    }
    
    // Callback for flying flag
    void flying_flag_cb(const std_msgs::Bool::ConstPtr &msg)
    {
        previous_flying_state = is_flying;
        is_flying = msg->data;

        // Subsequent requests: detect falling edge (flying -> not flying)
        if (first_request_done && previous_flying_state && !is_flying)
        {
            ROS_INFO("Vehicle has landed. Ready for next user input.");

            // Request user input
            requestUserInput();
        }
    }

    // Request user input and publish plan request
    void requestUserInput()
    {
        ROS_INFO("输入目标状态：");

        // Get target position
        geometry_msgs::Point target_pos = getTargetPositionFromUser();

        // Get target orientation
        geometry_msgs::Quaternion target_orientation = getTargetOrientationFromUser();

        // Get target velocity
        geometry_msgs::Vector3 target_vel = getTargetVelocityFromUser();

        // Get target angular velocity
        geometry_msgs::Vector3 target_angular_vel = getTargetAngularVelocityFromUser();

        // Get duration
        double duration = getDurationFromUser();

        // Package and publish plan request
        publishPlanRequest(target_pos, target_orientation, target_vel, target_angular_vel, duration);
    }

    // 交互式获取目标位置
    geometry_msgs::Point getTargetPositionFromUser()
    {
        geometry_msgs::Point target;
        while (true)
        {
            std::cout << "输入目标位置 (x y z，单位：米): ";
            std::cin >> target.x >> target.y >> target.z;

            if (std::cin.fail())
            { // 输入有效性检查
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "错误：请输入有效的数字！" << std::endl;
            }
            else
            {
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                break;
            }
        }
        return target;
    }

    // 交互式获取目标姿态
    geometry_msgs::Quaternion getTargetOrientationFromUser()
    {
        float roll, pitch, yaw;
        geometry_msgs::Quaternion res;
        while (true)
        {
            std::cout << "输入目标滚转、俯仰角、偏航角 (度)(世界坐标系X -> Y -> Z外旋): ";
            std::cin >> roll >> pitch >> yaw;

            if (std::cin.fail())
            {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "错误：请输入有效的角度值！" << std::endl;
            }
            else
            {
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                break;
            }
        }
        // 将输入的角度转换为四元数
        tf2::Quaternion q;
        q.setRPY(roll * M_PI / 180.0, pitch * M_PI / 180.0, yaw * M_PI / 180.0);
        q.normalize();
        res = tf2::toMsg(q);
        return res;
    }

    // 交互式获取目标线速度
    geometry_msgs::Vector3 getTargetVelocityFromUser()
    {
        geometry_msgs::Vector3 vel;
        std::string input;

        std::cout << "是否设置目标线速度? (y/n，默认为零): ";
        std::getline(std::cin, input);

        if (input == "y" || input == "Y")
        {
            while (true)
            {
                std::cout << "输入目标线速度 (vx vy vz，单位：米/秒): ";
                std::cin >> vel.x >> vel.y >> vel.z;

                if (std::cin.fail())
                {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "错误：请输入有效的数字！" << std::endl;
                }
                else
                {
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    break;
                }
            }
        }
        else
        {
            vel.x = 0.0;
            vel.y = 0.0;
            vel.z = 0.0;
        }
        return vel;
    }

    // 交互式获取目标角速度
    geometry_msgs::Vector3 getTargetAngularVelocityFromUser()
    {
        geometry_msgs::Vector3 ang_vel;
        std::string input;

        std::cout << "是否设置目标角速度? (y/n，默认为零): ";
        std::getline(std::cin, input);

        if (input == "y" || input == "Y")
        {
            while (true)
            {
                std::cout << "输入目标角速度 (wx wy wz，单位：弧度/秒): ";
                std::cin >> ang_vel.x >> ang_vel.y >> ang_vel.z;

                if (std::cin.fail())
                {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "错误：请输入有效的数字！" << std::endl;
                }
                else
                {
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    break;
                }
            }
        }
        else
        {
            ang_vel.x = 0.0;
            ang_vel.y = 0.0;
            ang_vel.z = 0.0;
        }
        return ang_vel;
    }

    // 交互式获取执行时间
    double getDurationFromUser()
    {
        double duration = 0.0;
        while (true)
        {
            std::cout << "输入总执行时间 (秒): ";
            std::cin >> duration;

            if (std::cin.fail() || duration <= 0)
            {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "错误：请输入有效的正数！" << std::endl;
            }
            else
            {
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                break;
            }
        }
        return duration;
    }

    // Publish plan request
    void publishPlanRequest(const geometry_msgs::Point &pos,
                            const geometry_msgs::Quaternion &orientation,
                            const geometry_msgs::Vector3 &vel,
                            const geometry_msgs::Vector3 &ang_vel,
                            double duration)
    {
        landing_planner::PlanRequest plan_req;

        plan_req.header.stamp = ros::Time::now();
        plan_req.header.frame_id = "map";

        // Set pose
        plan_req.pos.position = pos;
        plan_req.pos.orientation = orientation;

        // Set velocity
        plan_req.vel.linear = vel;
        plan_req.vel.angular = ang_vel;

        // Set duration
        plan_req.duration = duration;

        // Publish
        plan_request_pub.publish(plan_req);

        // Extract yaw for display
        tf2::Quaternion q;
        tf2::fromMsg(orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        ROS_INFO("Plan request published:");
        ROS_INFO("  Position: (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z);
        ROS_INFO("  Velocity: (%.2f, %.2f, %.2f)", vel.x, vel.y, vel.z);
        ROS_INFO("  Orientation (RPY): (%.2f, %.2f, %.2f) deg", roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
        ROS_INFO("  Angular Velocity: (%.2f, %.2f, %.2f) rad/s", ang_vel.x, ang_vel.y, ang_vel.z);
        ROS_INFO("  Duration: %.2f seconds", duration);
        ROS_INFO("Waiting for planner to execute...\n");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "user_input_node");

    // ROS_INFO("User Input Node started.");

    UserInputNode node;
    node.run();

    return 0;
}
