# One Cartesian waypoint of the flexiv_control action contract.
# Mirrors flexiv_control.action_chunk.CartesianWaypoint.
float64[3] position            # [x, y, z] in the base frame (metres)
float64[4] quaternion          # [qw, qx, qy, qz]; ignored if hold_orientation
bool hold_orientation          # true => keep current orientation (quaternion unused)
float64 gripper                # normalised gripper command in [0,1]; 1 = open; <0 = no change
bool has_gripper               # false => leave the gripper untouched this waypoint
int32 n_frames                 # number of low-level control frames; duration = n_frames / control_hz
float64 duration               # alternative to n_frames (seconds); used if n_frames <= 0
