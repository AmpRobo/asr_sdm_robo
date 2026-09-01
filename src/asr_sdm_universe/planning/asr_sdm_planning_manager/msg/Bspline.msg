int32 order
int64 traj_id
builtin_interfaces/Time start_time

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt

# Heading of a nonholonomic robot follows the trajectory tangent, so pitch is
# planned together with the position instead of being derived downstream.
float64[] pitch_pts
float64 pitch_dt
