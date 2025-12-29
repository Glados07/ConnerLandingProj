#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>

#include <landing_planner/PlanRequest.h>
#include <landing_planner/Trajectory.h>
#include <landing_planner/motion_generator.h>

class Planner {
private:
    ros::NodeHandle nh;
    ros::Rate rate;

// Subscribers
    mavros_msgs::State curr_state;
    ros::Subscriber state_sub;

    geometry_msgs::PoseStamped local_pos;
    ros::Subscriber local_pos_sub;

    geometry_msgs::TwistStamped local_vel;
    ros::Subscriber local_vel_sub;

    // geometry_msgs::Vector3 local_acc_raw;   // with gravity
    geometry_msgs::Vector3 local_acc;       // without gravity
    double acc_norm;
    ros::Subscriber imu_data_sub;

    bool plan_request_received;
    landing_planner::PlanRequest plan_request;
    ros::Subscriber plan_request_sub;

    // Takeoff state management
    bool takeoff_completed;
    ros::Time exec_start_time;

// Publishers
    ros::Publisher planned_traj_pub;
    ros::Publisher att_motion_cmd_pub;
    ros::Publisher tkof_cmd_pub;

public:
    Planner(float rate_hz) : rate(rate_hz), plan_request_received(false), takeoff_completed(false){
    // Subscribers
        state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &Planner::state_cb, this);
        local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Planner::local_pos_cb, this);
        local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, &Planner::local_vel_cb, this);
        imu_data_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, &Planner::imu_data_cb, this);
        plan_request_sub = nh.subscribe<landing_planner::PlanRequest>("plan_request", 10, &Planner::plan_request_cb, this);
    // Publishers
        planned_traj_pub = nh.advertise<landing_planner::Trajectory>("planned_trajectory", 10);
        att_motion_cmd_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
        tkof_cmd_pub = nh.advertise<std_msgs::Bool>("tkof_cmd", 10);
    }

    void run(){
        wait_for_arm();
        main_loop();
    }
private:
    // Callbacks
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        curr_state = *msg;
    }
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        local_pos = *msg;
    }
    void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
        local_vel.twist.linear = msg->twist.linear;
    }
    void imu_data_cb(const sensor_msgs::Imu::ConstPtr& msg){
        local_vel.twist.angular = msg->angular_velocity;
        // local_acc_raw = msg->linear_acceleration;
        Eigen::Quaterniond q_imu(msg->orientation.w, 
                                msg->orientation.x, 
                                msg->orientation.y, 
                                msg->orientation.z);

        Eigen::Vector3d acc_body(msg->linear_acceleration.x, 
                                msg->linear_acceleration.y, 
                                msg->linear_acceleration.z);

        // 旋转加速度到世界坐标系: acc_world = q * acc_body
        Eigen::Vector3d acc_world = q_imu * acc_body;

        // 去除重力偏置 (NED坐标系，重力向下为+9.81)    // MISCParam::g_acc
        const double GRAVITY = 9.81;
        acc_world.z() -= GRAVITY;

        // 保存去除重力后的加速度
        local_acc.x = acc_world.x();
        local_acc.y = acc_world.y();
        local_acc.z = acc_world.z();

        acc_norm = sqrt(local_acc.x * local_acc.x +
                        local_acc.y * local_acc.y +
                        local_acc.z * local_acc.z);
    }

    void plan_request_cb(const landing_planner::PlanRequest::ConstPtr& msg){
        plan_request_received = true;
        plan_request = *msg;
        ROS_INFO("Planner: Received new plan request.");

        // Construct target pose from PlanRequest
        geometry_msgs::PoseStamped target_pos;
        target_pos.pose = plan_request.pos;
        
        // Construct target velocity from PlanRequest
        geometry_msgs::TwistStamped target_vel;
        target_vel.twist = plan_request.vel;
        
        // Calculate target acceleration
        geometry_msgs::Vector3 target_acc = calculate_end_thrust_acc(target_pos);
        double duration = plan_request.duration;

        // First-time takeoff phase
        if (!takeoff_completed) {
            ROS_INFO("Planner: Starting takeoff phase...");
            exec_start_time = ros::Time::now();
            
            // Takeoff parameters (TODO: move to parameter server)
            const double TAKEOFF_IMPULSE_TIME = 0.5;      // seconds
            
            ros::Rate takeoff_rate(50.0);  // 50Hz for takeoff control
            std_msgs::Bool tkof_cmd;
            
            while(ros::ok()) {
                double elapsed_time = (ros::Time::now() - exec_start_time).toSec();
                
                // Check if takeoff phase is complete
                if(elapsed_time >= TAKEOFF_IMPULSE_TIME) {
                    ROS_INFO("Planner: Takeoff phase completed at t=%.2fs", elapsed_time);
                    takeoff_completed = true;
                    
                    // Send stop takeoff command
                    tkof_cmd.data = false;
                    tkof_cmd_pub.publish(tkof_cmd);
                    break;
                }
                
                // Send takeoff command to px4_cmd_sender
                tkof_cmd.data = true;
                tkof_cmd_pub.publish(tkof_cmd);
                
                ROS_INFO_THROTTLE(0.5, "Planner: Sending takeoff command [%.2fs]", elapsed_time);
                
                takeoff_rate.sleep();
            }
        }

        // Motion generation with replanning
        motion::MotionGenerator motion_gen;
        
        // Fixed-frequency replanning
        const double replanning_frequency = 1.0; // Hz
        const double dt_replan = 1.0 / replanning_frequency;
        ros::Rate replanning_rate(replanning_frequency);

        while (duration > 1.0) {
            motion_gen.generateMotionFrameSet(local_pos,local_vel,local_acc,
                                            target_pos,target_vel,target_acc,
                                            duration);
            // Get polynomial coefficients
            auto x_coeffs = motion_gen.getXCoeffs();
            auto y_coeffs = motion_gen.getYCoeffs();
            auto z_coeffs = motion_gen.getZCoeffs();
            auto yaw_coeffs = motion_gen.getYawCoeffs();
            
            // Package trajectory message
            landing_planner::Trajectory traj_msg;
            traj_msg.header.stamp = ros::Time::now();
            traj_msg.header.frame_id = "map";
            
            // Copy coefficients
            std::copy(x_coeffs.begin(), x_coeffs.end(), traj_msg.x_coeffs.begin());
            std::copy(y_coeffs.begin(), y_coeffs.end(), traj_msg.y_coeffs.begin());
            std::copy(z_coeffs.begin(), z_coeffs.end(), traj_msg.z_coeffs.begin());
            std::copy(yaw_coeffs.begin(), yaw_coeffs.end(), traj_msg.yaw_coeffs.begin());
            
            traj_msg.duration = duration;
            
            // Publish trajectory
            planned_traj_pub.publish(traj_msg);
            duration -= dt_replan;
            replanning_rate.sleep();
        }
        ROS_INFO("Planner: Plan request processing complete.");
        plan_request_received = false;
    }

    void wait_for_arm() {
        ROS_INFO("Planner Node: Waiting for vehicle to be armed...");
        ros::Time start_time = ros::Time::now();
        while (ros::ok()){
            if (curr_state.armed && curr_state.mode == "OFFBOARD"){
                return;
            }
            // Overtime protection
            if ((ros::Time::now() - start_time) > ros::Duration(60.0)){
                ROS_WARN("Planner Node: Wait for arm timeout after 60 seconds. Exiting...");
                ros::shutdown();
                return;
            }
            rate.sleep();
        }
    }
    void main_loop() {
        ROS_INFO("Planner Node: Vehicle armed. Starting main loop.");
        while (ros::ok()){
            // Main planning loop - process requests as they come
            rate.sleep();
        }
    }
private:
    // Helper function: Convert geometry_msgs::Quaternion to Eigen::Matrix3d
    Eigen::Matrix3d geoQuat_to_EigenMat3(const geometry_msgs::Quaternion &quat) {
        Eigen::Quaterniond eigen_quat(quat.w, quat.x, quat.y, quat.z);
        return eigen_quat.toRotationMatrix();
    }

    // Calculate end thrust acceleration to balance gravity
    geometry_msgs::Vector3 calculate_end_thrust_acc(const geometry_msgs::PoseStamped &end_pos) {
        // TODO： MiscParams
        const double GRAVITY = 9.81;
        const double terminal_z_acc_offset = 0.0;    // No offset for now
        
        Eigen::Matrix3d R_mat = geoQuat_to_EigenMat3(end_pos.pose.orientation);
        Eigen::Vector3d Z_b2w = R_mat.col(2); //Z_b project to world frame
        double t_norm = (GRAVITY + terminal_z_acc_offset) / Z_b2w.dot(Eigen::Vector3d(0, 0, 1));  //Calculate thrust acc to balance the gravity acc
        Eigen::Vector3d ddsigma_123 = t_norm * Z_b2w - Eigen::Vector3d(0, 0, GRAVITY);
        
        geometry_msgs::Vector3 result;
        result.x = ddsigma_123[0]; //ddx
        result.y = ddsigma_123[1]; //ddy
        result.z = ddsigma_123[2]; //ddz
        
        return result;
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "landing_planner_node");
    ros::NodeHandle nh;

    Planner planner(50.0); // 50Hz control loop

    // Multithreading for callbacks
    ros::AsyncSpinner spinner(2); // Use 2 threads for callbacks
    spinner.start();
    planner.run();

    return 0;
}




