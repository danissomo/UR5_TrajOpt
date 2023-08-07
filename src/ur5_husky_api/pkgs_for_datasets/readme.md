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
q_target: [1.0789810419082357, -0.5552123188973042, 1.9236936569214151, -3.0894486904144287, -1.5335940122604406, 0.0014620755100622773]
qd_target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
i_target: [1.5007602085703763e-16, -4.107097148331976, -0.7316962346134158, -0.4722812344084691, 0.014116328767848943, 0.0]
m_target: [1.7871342335038467e-15, -49.07708268268306, -8.81474625524768, -3.9657823467345623, 0.1178101831045303, 0.0]
tau_target: [-2.8435192108154297, -4.686834335327148, -2.524993658065796, -1.8476637601852417, -0.09623318165540695, 0.27296027541160583]
tool_vector_target: [-0.1620637254951934, -0.5559066938963935, -0.27617376149062756, -0.747626418364051, -3.0410411240398885, 0.24486997021546575]
q_actual: [1.0790050029754639, -0.5552361647235315, 1.9236574172973633, -3.0895207563983362, -1.5335095564471644, 0.0015100124292075634]
qd_actual: [0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
i_actual: [0.19503751397132874, -3.755032777786255, -0.54475998878479, -0.28060758113861084, 0.024400658905506134, -0.027450740337371826]
tau_actual: [1.7871342335038467e-15, -49.07708268268306, -8.81474625524768, -3.9657823467345623, 0.1178101831045303, 0.0]
tcp_force: [-3.9507139576431167, 1.4684239892480926, 6.419865146104576, -1.1559420904772162, -0.3154261897823234, -0.1696652404525002]
tool_vector_actual: [-0.16205484784543783, -0.5559588626354566, -0.27614295197823163, -0.7476583105626016, -3.041071288494682, 0.24510633909253016]
tcp_speed: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
motor_temperatures: [36.359981536865234, 35.7199821472168, 32.819984436035156, 41.178184509277344, 41.89998245239258, 43.379981994628906]
joint_modes: [253.0, 253.0, 253.0, 253.0, 253.0, 253.0]
controller_timer: 0.616656
qdd_target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
qdd_actual: []
tool_acc_values: [-0.2607249319553375, -1.2736562490463257, -10.027420997619629]
robot_mode: 7.0
digital_input_bits: 1.0
test_value: 0.0
```


### Ошибки

При параллельном подключении возникает ошибка ```[FATAL] [1691406822.304592]: One of the RTDE input registers are already in use! Currently you must disable the EtherNet/IP adapter, PROFINET or any MODBUS unit configured on the robot. This might change in the future.
```



