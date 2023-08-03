## Здесь лежат пакеты для сбора датасета

### arm_msgs
- Type of the messages - **arm_msgs/ManipulatorState**
- Name of the topic - **/state/arm/0/arm_state** 

Types: 
- float64[] q_target: An array of floating-point numbers representing the target joint positions.
- float64[] qd_target: An array of floating-point numbers representing the target joint velocities.
- float64[] i_target: An array of floating-point numbers representing the target joint currents.
- float64[] m_target: An array of floating-point numbers representing the target joint moments.
- float64[] tau_target: An array of floating-point numbers representing the target joint torques.
- float64[] tool_vector_target: An array of floating-point numbers representing the tool vector.
- float64[] q_actual: An array of floating-point numbers representing the actual joint positions.
- float64[] qd_actual: An array of floating-point numbers representing the actual joint velocities.
- float64[] i_actual: An array of floating-point numbers representing the actual joint currents.
- float64[] tau_actual: An array of floating-point numbers representing the actual joint torques.
- float64[] tcp_force: An array of floating-point numbers representing the force applied to the tool center point (TCP).
- float64[] tool_vector_actual: An array of floating-point numbers representing the tool vector.
- float64[] tcp_speed: An array of floating-point numbers representing the speed of the TCP.
- float64[] motor_temperatures: An array of floating-point numbers representing the temperatures of the motors.
- float64[] joint_modes: An array of floating-point numbers representing the modes of the joints.
- float64 controller_timer: A floating-point number representing the controller timer.
- float64[] qdd_target: An array of floating-point numbers representing the target joint accelerations.
- float64[] qdd_actual: An array of floating-point numbers representing the actual joint accelerations.
- float64[] tool_acc_values: An array of floating-point numbers representing the tool acceleration values.
- float64 robot_mode: A floating-point number representing the current robot mode.
- float64 digital_input_bits: A floating-point number representing the state of the digital input bits.
- float64 test_value: A floating-point number representing a test value.



