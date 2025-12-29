#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>

#include <eigen3/Eigen/Dense>

namespace DFlatness
{
    constexpr double g_acc = 9.81; //MISCParam::g_acc;
    /*###################################################*/
    /*################ Inverse Transform ################*/
    /*###################################################*/
    // Differential Flatness Constraints
    struct DiffConstraintsType{
        double x,y,z;           // Position x,y,z
        double vx,vy,vz;        // Velocity x,y,z
        double ddx,ddy,ddz;     // Acceleration x,y,z
        double dddx,dddy,dddz;  // Jerk x,y,z
        double psi;             // Yaw angle
        double dpsi;            // Yaw rate
    };

    inline double quat_to_yaw(const geometry_msgs::Quaternion &q) {
        return atan2(2.0 * (q.w * q.z + q.x * q.y), 
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    inline Eigen::Matrix3d quat_to_rotmat(const geometry_msgs::Quaternion &geo_q) {
        Eigen::Quaterniond q(geo_q.w, geo_q.x, geo_q.y, geo_q.z);
        return q.toRotationMatrix();
    }

    // Inverse flatness transform: State -> Differential Flatness Constraints
    inline DiffConstraintsType Inverse_flatness(const geometry_msgs::PoseStamped &pos,
                                                const geometry_msgs::TwistStamped &vel,
                                                const geometry_msgs::Vector3 &acc)
    {
        DiffConstraintsType constraints;
        
        // Extract position
        constraints.x = pos.pose.position.x;
        constraints.y = pos.pose.position.y;
        constraints.z = pos.pose.position.z;

        // Extract velocity
        constraints.vx = vel.twist.linear.x;
        constraints.vy = vel.twist.linear.y;
        constraints.vz = vel.twist.linear.z;

        // Extract acceleration (in world frame)
        constraints.ddx = acc.x;
        constraints.ddy = acc.y;
        constraints.ddz = acc.z;

        // Extract yaw angle directly from quaternion
        constraints.psi = quat_to_yaw(pos.pose.orientation);

        // Get angular velocity in body frame
        double p = vel.twist.angular.x;  // roll rate
        double q = vel.twist.angular.y;  // pitch rate
        double r = vel.twist.angular.z;  // yaw rate

        // Convert quaternion to rotation matrix (body frame axes in world frame)
        Eigen::Matrix3d R_mat = quat_to_rotmat(pos.pose.orientation);

        // Get body frame axes
        Eigen::Vector3d X_b = R_mat.col(0);  // Body x-axis in world frame
        Eigen::Vector3d Y_b = R_mat.col(1);  // Body y-axis in world frame
        Eigen::Vector3d Z_b = R_mat.col(2);  // Body z-axis in world frame

        // Compute thrust acceleration vector (including gravity compensation)
        Eigen::Vector3d thr_acc(constraints.ddx, constraints.ddy, constraints.ddz + g_acc);
        double thr_norm = thr_acc.norm();

        // Reconstruct h_omega from angular velocity
        Eigen::Vector3d h_omega = q * X_b - p * Y_b;

        // Calculate jerk from h_omega
        Eigen::Vector3d jerk = thr_norm * h_omega;

        constraints.dddx = jerk[0];
        constraints.dddy = jerk[1];
        constraints.dddz = jerk[2];

        // Calculate dpsi (yaw rate in world frame)
        double e3_dot_Zb = Z_b[2];
        if (fabs(e3_dot_Zb) > 1e-6) {
            constraints.dpsi = r / e3_dot_Zb;
        } else {
            // Near singularity (Z_b nearly horizontal)
            // Use approximation: dpsi ≈ r
            constraints.dpsi = r;
            ROS_WARN_THROTTLE(1.0, "Differential flatness near singularity (e3·Z_b = %.4f), using approximation for dpsi", e3_dot_Zb);
        }

        return constraints;
    }


    // /*###################################################*/
    // /*################ Forward Transform ################*/
    // /*###################################################*/
    // // Position, Velocity, Acceleration, Attitude, Angular velocity, Thurst Acc, Omega Acc
    // struct StateType{
    //     struct Position {
    //         double x;          // Position x
    //         double y;          // Position y
    //         double z;          // Position z
    //     } pos;          // Position
    //     struct Velocity{
    //         double x;          // Velocity x
    //         double y;          // Velocity y
    //         double z;          // Velocity z
    //     } vel;          // Velocity
    //     struct Acceleration{
    //         double x;          // Acceleration x
    //         double y;          // Acceleration y
    //         double z;          // Acceleration z
    //     } acc;          // Acceleration

    //     struct Attitude_Eular{
    //         double roll;       // Roll angle
    //         double pitch;      // Pitch angle
    //         double yaw;        // Yaw angle
    //     } att_eul;          // Attitude by Euler angles
    //     geometry_msgs::Quaternion att_quat; // Attitude by Quaternion

    //     struct Omega {
    //         double bx;   // Angular velocity around bx-axis
    //         double by;   // Angular velocity around by-axis
    //         double bz;   // Angular velocity around bz-axis
    //     } omega;     // Angular velocity

    //     //u1, u2, u3, u4 without counting mass & inertia
    //     double thrust_acc;    // magnitude of acceleration provided by thrust
    //     struct Omega_acc{
    //         double bx;      // Omega_acc around bx-axis
    //         double by;      // Omega_acc around by-axis
    //         double bz;      // Omega_acc around bz-axis
    //     } omega_acc; // Omega acceleration;         
    // };
    // using P_7od_CoeffsType = std::array<double, 8>;  // Coefficients for 7th order polynomial: p0, p1, p2, p3, p4, p5, p6, p7
    // using P_3od_CoeffsType = std::array<double, 4>;  // Coefficients for 3rd order polynomial: p0, p1, p2, p3
    // // Input: x,y,z,psi coefficients
    // // Output: State at time t

    // class DFTransformHandle
    // {
    // private:
    //     double PosX_AtTime_7od(const P_7od_CoeffsType& coeffs, const double &t);
    //     double VelX_AtTime_7od(const P_7od_CoeffsType& coeffs, const double &t);
    //     double AccX_AtTime_7od(const P_7od_CoeffsType& coeffs, const double &t);
    //     double JrkX_AtTime_7od(const P_7od_CoeffsType& coeffs, const double &t);
    //     double SnpX_AtTime_7od(const P_7od_CoeffsType& coeffs, const double &t);
        
    //     double PosX_AtTime_3od(const P_3od_CoeffsType& coeffs, const double &t);
    //     double VelX_AtTime_3od(const P_3od_CoeffsType& coeffs, const double &t);
    //     double AccX_AtTime_3od(const P_3od_CoeffsType& coeffs, const double &t);

    //     Eigen::Matrix3d Attitude_AtTime(Eigen::Vector3d thr_acc, double psi);
    //     Eigen::Vector3d Omega_AtTime(Eigen::Vector3d thr_acc, Eigen::Vector3d jerk, Eigen::Matrix3d RMatrix, double dpsi);
    //     Eigen::Vector3d OmegaAcc_AtTime(Eigen::Vector3d thr_acc, Eigen::Vector3d jerk, Eigen::Vector3d snap, 
    //                                   Eigen::Matrix3d RMatrix, Eigen::Vector3d omega_b, double ddpsi);
    // public:
    //     StateType DFTransformAtTime(
    //         const P_7od_CoeffsType &x_coeffs,
    //         const P_7od_CoeffsType &y_coeffs,
    //         const P_7od_CoeffsType &z_coeffs,
    //         const P_3od_CoeffsType &yaw_coeffs,
    //         const double &t);
    // };
};
