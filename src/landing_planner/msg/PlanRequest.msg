# Plan Request Message for Landing Planner
# Contains target end state: position, velocity, orientation, and angular velocity

Header header

# Target pose
geometry_msgs/Pose pos

# Target velocity
geometry_msgs/Twist vel

# # Target acc (not used currently)
# geometry_msgs/Accel acc

# Execution duration (seconds)
float64 duration
