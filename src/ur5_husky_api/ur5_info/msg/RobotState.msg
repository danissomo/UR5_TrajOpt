# param jointPositions -  actual position of robot joints
# param jointVelocitiesFact - actual joint velocities
# param jointVelocitiesPlan - planned joint velocity (joint speed of leading axis [rad/s]), value '-1' - the velocity is not defined
# param joint_acceleration_plan - joint acceleration of leading axis [rad/s^2], value '-1' - the velocity is not defined
float64[] joint_positions
float64[] joint_velocities_fact
float64 joint_velocity_plan
float64 joint_acceleration_plan