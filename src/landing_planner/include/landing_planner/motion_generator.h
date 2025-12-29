#pragma once
#include <ros/ros.h>
#include <array>
#include <eigen3/Eigen/Dense>
#include <vector>

# include <landing_planner/differential_flatness.h>
# include <landing_planner/PolyTraj.h>

namespace motion
{
    class MotionGenerator
    {
    private:
        using Vector8d = Eigen::Matrix<double, 8, 1>;
        using Vector4d = Eigen::Matrix<double, 4, 1>;
        using Matrix8d = Eigen::Matrix<double, 8, 8>;
        using Traj_7od_Constraints = Vector8d;  //X0, dX0, ddX0, dddX0, XT, dXT, ddXT, dddXT
        using Traj_3od_Constraints = Vector4d;  //X0, dX0, XT, dXT
    private:
        Traj_7od_Constraints x_constraints;
        Traj_7od_Constraints y_constraints;
        Traj_7od_Constraints z_constraints;
        Traj_3od_Constraints yaw_constraints;
    
        PolyTraj::Traj_7od x_coeffs;
        PolyTraj::Traj_7od y_coeffs;
        PolyTraj::Traj_7od z_coeffs;
        PolyTraj::Traj_3od yaw_coeffs;

        //7 order polynomial
        PolyTraj::Traj_7od generateCoeffs_7od(const Traj_7od_Constraints& vec_constraints, const double &T)
        {
            Vector8d coeffs;
            Matrix8d Q_T, Q_T_inv;
            Q_T <<  1, 0, 0, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0, 0, 0,
                    0, 0, 2, 0, 0, 0, 0, 0,
                    0, 0, 0, 6, 0, 0, 0, 0,
                    1, T, pow(T, 2),    pow(T, 3),          pow(T, 4),      pow(T, 5),      pow(T, 6),      pow(T, 7),
                    0, 1, 2*T,          3*pow(T, 2),        4*pow(T, 3),    5*pow(T, 4),    6*pow(T, 5),    7*pow(T, 6),
                    0, 0, 2,            6*T,                12*pow(T, 2),   20*pow(T, 3),   30*pow(T, 4),   42*pow(T, 5),
                    0, 0, 0,            6,                  24*T,           60*pow(T, 2),   120*pow(T, 3),  210*pow(T, 4);

            // Calculate coefficents
            if (Q_T.determinant() != 0) {
                Q_T_inv = Q_T.inverse();
                coeffs = Q_T_inv * vec_constraints;
            }
            else
            {
                ROS_ERROR("The matrix Q_T_xyz is singular, cannot calculate the coefficients.");
                coeffs.setZero();
            }
            PolyTraj::Traj_7od res = {coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7]};
            return res;
        }

        //3 order polynomial
        PolyTraj::Traj_3od generateCoeffs_3od(const Traj_3od_Constraints& vec_constraints,const double &T)
        {
            Eigen::Vector4d coeffs;
            Eigen::Matrix4d Q_T, Q_T_inv;
            Q_T <<  1, 0, 0,            0,
                    0, 1, 0,            0,
                    1, T, pow(T, 2),    pow(T, 3),
                    0, 1, 2*T,          3*pow(T, 2);

            // Calculate coefficents
            if (Q_T.determinant() != 0) {
                Q_T_inv = Q_T.inverse();
                coeffs = Q_T.inverse() * vec_constraints;
            }
            else
            {
                ROS_ERROR("The matrix Q_T_yaw is singular, cannot calculate the coefficients.");
                coeffs.setZero();
            }

            PolyTraj::Traj_3od res = {coeffs[0], coeffs[1], coeffs[2], coeffs[3]};
            return res;
        }
    public:
        // Generate motion frame set from start & end states
        // Notice: pos, vel include linear & angular parts, where as acc only includes linear part
        // Return true if feasible, false otherwise
        bool generateMotionFrameSet(const geometry_msgs::PoseStamped& start_pos, 
                            const geometry_msgs::TwistStamped& start_vel,
                            const geometry_msgs::Vector3& start_acc,
                            const geometry_msgs::PoseStamped& end_pos, 
                            const geometry_msgs::TwistStamped& end_vel,
                            const geometry_msgs::Vector3& end_acc,
                            const double &duration)
        {
            DFlatness::DiffConstraintsType start_constraints = DFlatness::Inverse_flatness(start_pos, start_vel, start_acc);
            DFlatness::DiffConstraintsType end_constraints = DFlatness::Inverse_flatness(end_pos, end_vel, end_acc);

            x_constraints[0] = start_constraints.x;         //X0
            x_constraints[1] = start_constraints.vx;        //dX0
            x_constraints[2] = start_constraints.ddx;       //ddX0
            x_constraints[3] = start_constraints.dddx;      //dddX0
            x_constraints[4] = end_constraints.x;           //XT
            x_constraints[5] = end_constraints.vx;          //dXT
            x_constraints[6] = end_constraints.ddx;         //ddXT
            x_constraints[7] = end_constraints.dddx;        //dddXT
            
            y_constraints[0] = start_constraints.y;         //Y0
            y_constraints[1] = start_constraints.vy;        //dY0
            y_constraints[2] = start_constraints.ddy;       //ddY0
            y_constraints[3] = start_constraints.dddy;      //dddY0
            y_constraints[4] = end_constraints.y;           //YT
            y_constraints[5] = end_constraints.vy;          //dYT
            y_constraints[6] = end_constraints.ddy;         //ddYT
            y_constraints[7] = end_constraints.dddy;        //dddYT
            
            z_constraints[0] = start_constraints.z;         //Z0
            z_constraints[1] = start_constraints.vz;        //dZ0
            z_constraints[2] = start_constraints.ddz;       //ddZ0
            z_constraints[3] = start_constraints.dddz;      //dddZ0
            z_constraints[4] = end_constraints.z;           //ZT
            z_constraints[5] = end_constraints.vz;          //dZT
            z_constraints[6] = end_constraints.ddz;         //ddZT
            z_constraints[7] = end_constraints.dddz;        //dddZT
            
            yaw_constraints[0] = start_constraints.psi;     //psi0
            yaw_constraints[1] = start_constraints.dpsi;    //dpsi0
            yaw_constraints[2] = end_constraints.psi;       //psiT
            yaw_constraints[3] = end_constraints.dpsi;      //dpsiT

            x_coeffs = generateCoeffs_7od(x_constraints, duration);
            y_coeffs = generateCoeffs_7od(y_constraints, duration);
            z_coeffs = generateCoeffs_7od(z_constraints, duration);
            yaw_coeffs = generateCoeffs_3od(yaw_constraints, duration);
            if(1){  // TODO: add feasibility check here
                return true;
            }
            else {  // If not feasible return false
                return false;
            }
        }        
        // TODO: Feasibility Checker
        // double getMaxAcc
        // double getMaxVel
        // double getMaxPos...
        PolyTraj::Traj_7od getXCoeffs() const { return x_coeffs; };
        PolyTraj::Traj_7od getYCoeffs() const { return y_coeffs; };
        PolyTraj::Traj_7od getZCoeffs() const { return z_coeffs; };
        PolyTraj::Traj_3od getYawCoeffs() const { return yaw_coeffs; };
    };
}