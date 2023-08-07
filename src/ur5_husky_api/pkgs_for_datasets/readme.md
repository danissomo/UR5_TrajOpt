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
- Запустить сбор положения робота в датасет ```roslaunch arm_msgs run.launch```

### Ситуация 1 (робот не движется и находится в сложенном состоянии)

**Request**

```rostopic echo /state/arm/0/arm_state```

**Response**

```
q_target: [1.4775620698928549, -1.3088954687117962, 1.4463391304016397, -2.300877809524536, -1.580016016960144, 0.00014381069922808631]
qd_target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
i_target: [8.327255444779821e-18, -3.4215973902831296, -2.3755720623936036, -0.5614347212770948, -0.0029354753525478706, 0.0]
m_target: [9.916256568845089e-17, -40.88581593400494, -28.618522209723398, -4.714411126822055, -0.024498500599543484, 0.0]
tau_target: [-4.310176372528076, -9.258805274963379, -5.652396202087402, -1.725041389465332, -0.18444034457206726, 0.02873266115784645]
tool_vector_target: [0.04221340650094885, -0.7085035701843826, 0.31436791704317824, -0.13442592182137317, -2.96116811739825, 0.905430372159358]
q_actual: [1.4775381088256836, -1.3089678923236292, 1.4463152885437012, -2.3008659521686, -1.580099884663717, 7.190534961409867e-05]
qd_actual: [0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
i_actual: [0.2891935706138611, -2.7148325443267822, -1.9436497688293457, -0.37058499455451965, 0.0091502470895648, 0.0]
tau_actual: [9.916256568845089e-17, -40.88581593400494, -28.618522209723398, -4.714411126822055, -0.024498500599543484, 0.0]
tcp_force: [-7.146445469989163, 6.5405868229671595, 11.20715875304465, -0.41725804807280387, -1.3301808778383377, 0.9261053922341322]
tool_vector_actual: [0.04217908282225835, -0.7084894524815342, 0.31443070328351874, -0.1343873338378248, -2.9610267165175572, 0.90551488399014]
tcp_speed: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
motor_temperatures: [36.46983337402344, 35.82998275756836, 32.89935302734375, 41.2199821472168, 41.9699821472168, 43.50998306274414]
joint_modes: [253.0, 253.0, 253.0, 253.0, 253.0, 253.0]
controller_timer: 0.567756
qdd_target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
qdd_actual: []
tool_acc_values: [0.31466802954673767, -5.235476493835449, -8.508024215698242]
robot_mode: 7.0
digital_input_bits: 1.0
test_value: 0.0
```

### Ситуация 2 (робот движется)

```
q_target: [1.4932261471357287, -1.5159815676706607, 1.6252961875759546, -2.4141879503789574, -1.5801272883096402, 0.00012160387748207902]
qd_target: [-0.007564041022074667, 0.10000000149011612, -0.0864167419931981, 0.054716440703928226, 5.373192682329287e-05, 1.07234730813308e-05]
i_target: [-0.39282497552185835, -2.0809956928730333, -2.9066561178636756, -0.37858138398385177, -0.0017470507702022449, 0.00021223162360249796]
m_target: [-0.0014532981152663004, -31.33629992622082, -28.739289376216643, -4.762172804071717, -0.022385009689536783, 0.0]
tau_target: []
tool_vector_target: [0.05881841929377361, -0.6390470281309372, 0.36912848986011065, -0.1090437394418686, -2.890260481715334, 1.1128193796485126]
q_actual: [1.4932153224945068, -1.5160635153399866, 1.6253647804260254, -2.414271656666891, -1.580099884663717, 0.00015579492901451886]
qd_actual: [-0.00960016343742609, 0.10190746188163757, -0.08600518852472305, 0.053713299334049225, 0.0005992112564854324, -2.3968450477696024e-05]
i_actual: [-0.6052888631820679, -1.6275544166564941, -2.295614004135132, -0.30043309926986694, 0.12352833896875381, -0.053376439958810806]
tau_actual: [-0.0014532981152663004, -31.33629992622082, -28.739289376216643, -4.762172804071717, -0.022385009689536783, 0.0]
tcp_force: [9.186360179467217, -6.881280204701753, 18.59290642207218, 4.116703439633063, 1.7609472492463485, -2.6288142296605064]
tool_vector_actual: [0.058819610709469845, -0.6390238783949221, 0.3691568366201661, -0.10909291118045662, -2.89023256762675, 1.112972587862422]
tcp_speed: [-0.008847712357707612, -0.03597751775326813, -0.0238633777039811, 0.06938391699549044, -0.005724054913938763, -0.009556450308965502]
motor_temperatures: [36.44001770019531, 34.2800178527832, 33.51001739501953, 41.32998275756836, 42.60001754760742, 44.270015716552734]
joint_modes: [253.0, 253.0, 253.0, 253.0, 253.0, 253.0]
controller_timer: 0.404844
qdd_target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
qdd_actual: []
tool_acc_values: [0.4525225758552551, -6.4791646003723145, -7.53704833984375]
robot_mode: 7.0
digital_input_bits: 1.0
test_value: 0.0

```


### Ошибки

При параллельном подключении возникает ошибка ```[FATAL] [1691406822.304592]: One of the RTDE input registers are already in use! Currently you must disable the EtherNet/IP adapter, PROFINET or any MODBUS unit configured on the robot. This might change in the future.
```



