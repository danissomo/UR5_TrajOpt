# Здесь лежат пакеты для сбора датасета

## arm_msgs
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

Запуск:
- Перекинуть пакет на робота в папку ```~/rubleva/ur5_husky_api```
- Перейти в папку проекта ```cd ~/rubleva/ur5_husky_api```
- Собрать проект ```catkin_make```
- Прописать пути ```source devel/setup.bash```
- Запустить сбор положения робота в датасет ```roslaunch arm_msgs robot_info.launch```

### Ситуация 1 (робот не движется и находится в сложенном состоянии)

**Request**

```rostopic echo /state/arm/0/arm_state```

**Response**

```
q_target: [1.5960314273834229, -2.8722265402423304, 2.789100408554077, -3.189812485371725, -1.5799196402179163, 0.0003954794374294579]
qd_target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
i_target: [3.79090568175729e-18, 1.5823841366451916, -2.2214900833659685, -0.32320019471202366, 0.000458649921594441, 0.0]
m_target: [4.514283681805781e-17, 18.9084392954868, -26.762296246837884, -2.713937233300185, 0.0038277396433969027, 0.0]
tau_target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
tool_vector_target: [0.11419276827756972, -0.1861222031822081, 0.35793303252928277, -0.02536864521331505, 2.0635144422063334, -2.3579359600272505]
q_actual: [1.596043348312378, -2.8722384611712855, 2.789088249206543, -3.189812485371725, -1.579991642628805, 0.00032357408781535923]
qd_actual: [0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
i_actual: [-0.22866468131542206, 1.4347587823867798, -1.7934484481811523, -0.18758006393909454, 0.042701154947280884, 0.0732019767165184]
tau_actual: [4.514283681805781e-17, 18.9084392954868, -26.762296246837884, -2.713937233300185, 0.0038277396433969027, 0.0]
tcp_force: [-182.89498502808993, -25.1694216452617, 13.273960572338751, -1.1769906817946059, 6.02084709194934, 39.966544220755466]
tool_vector_actual: [0.11417884153265309, -0.18611602208124317, 0.3579426837284969, -0.02537053005740153, 2.0635459009933212, -2.3580312330467725]
tcp_speed: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
motor_temperatures: [30.619468688964844, 28.364500045776367, 28.037174224853516, 32.529483795166016, 31.349987030029297, 32.89188766479492]
joint_modes: [5.36864935239e-312, 5.36864935239e-312, 5.36864935239e-312, 5.36864935239e-312, 5.36864935239e-312, 7.0025861227e-313]
controller_timer: 0.367416
qdd_target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
qdd_actual: []
tool_acc_values: [0.6293360590934753, -9.616854667663574, 0.35362693667411804]
robot_mode: 1.27319747493e-313
digital_input_bits: 4.94065645841e-324
test_value: 0.0
```

### Ситуация 2 (робот движется)








