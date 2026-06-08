# Snapshot of the robot, mirroring flexiv_control.types.RobotState.
std_msgs/Header header
float64[] q                    # joint positions (rad)
float64[] dq                   # joint velocities (rad/s)
float64[7] tcp_pose            # [x,y,z, qw,qx,qy,qz]
float64[6] tcp_vel             # [vx,vy,vz, wx,wy,wz]
float64[6] wrench              # external wrench, base frame, estimated at the TCP [fx,fy,fz, tx,ty,tz]
float64 gripper_width          # metres
bool gripper_is_moving
string control_mode            # e.g. "NRT_CARTESIAN_MOTION_FORCE"
string safety_status           # e.g. "OK" (OK/CLIPPED/HOLDING/STOPPED/FAULT)
