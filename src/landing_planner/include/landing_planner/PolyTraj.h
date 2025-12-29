# pragma once
# include <array>
# include <cmath>

namespace PolyTraj{
    typedef std::array<double, 8> Traj_7od;  //p0, p1, p2, p3, p4, p5, p6, p7
    typedef std::array<double, 4> Traj_3od;  //p0, p1, p2, p3
    
    double PosX_AtTime_7od(const Traj_7od& coeffs, const double &t){
        return coeffs[0] + coeffs[1] * t + coeffs[2] * pow(t, 2) + coeffs[3] * pow(t, 3) +
           coeffs[4] * pow(t, 4) + coeffs[5] * pow(t, 5) + coeffs[6] * pow(t, 6) + coeffs[7] * pow(t, 7);
    };
    double VelX_AtTime_7od(const Traj_7od& coeffs, const double &t){
        return coeffs[1] + 2 * coeffs[2] * t + 3 * coeffs[3] * pow(t, 2) +
           4 * coeffs[4] * pow(t, 3) + 5 * coeffs[5] * pow(t, 4) + 6 * coeffs[6] * pow(t, 5) + 7 * coeffs[7] * pow(t, 6);
    };
    double AccX_AtTime_7od(const Traj_7od& coeffs, const double &t){
        return 2 * coeffs[2] + 6 * coeffs[3] * t +
           12 * coeffs[4] * pow(t, 2) + 20 * coeffs[5] * pow(t, 3) + 30 * coeffs[6] * pow(t, 4) + 42 * coeffs[7] * pow(t, 5);
    };
    // double JrkX_AtTime_7od(const Traj_7od& coeffs, const double &t);
    // double SnpX_AtTime_7od(const Traj_7od& coeffs, const double &t);
    
    double PosX_AtTime_3od(const Traj_3od& coeffs, const double &t){
        return coeffs[0] + coeffs[1] * t + coeffs[2] * pow(t, 2) + coeffs[3] * pow(t, 3);
    };
    double VelX_AtTime_3od(const Traj_3od& coeffs, const double &t){
        return coeffs[1] + 2 * coeffs[2] * t + 3 * coeffs[3] * pow(t, 2);
    };
    // double AccX_AtTime_3od(const Traj_3od& coeffs, const double &t);
}