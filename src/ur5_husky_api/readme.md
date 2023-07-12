## Здесь находятся пакеты, которые необходимо запускать на роботе

При создании новых пакетов для робота необходимо добавлять файл <code>CATKIN_IGNORE</code>, чтобы игнорировать их при сборке проекта.

После загрузки пакетов на робота удалить файл <code>CATKIN_IGNORE</code>.

Путь до пакета на роботе: <code>/home/administrator/rubleva/ur5_husky_api</code>.


===================


source devel/setup.bash

====================

TOPICS_FOR_LLM in recorder.py


### 0. Запустить топики для rosbags

1 терминал
roslaunch ur5_info ur5_info.launch


Управление гриппером
Открыть терминал
cd /rubleva/ur5_husky_api
rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.04" --once


Управление манипулятором
ВАЖНО!!!
position и delay - передавать массив строго 6 элементов


// положение берет куб слева
cd /rubleva/ur5_husky_api
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[2.326473,-0.553581,1.686786,-2.687753,-1.561592,-0.000419]], delay: [0,0,0,0,0,20], gripperAngle: 0.0}" --once


// конечное положение
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.606798,-3.091649,2.827192,-1.962667,-1.436540,-0.000551]], delay: [0,0,0,0,0,20], gripperAngle: 0.0}" --once

// позиция Даниила
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.595672, -2.871760, 2.799204, -3.072348,-1.581982, 0.000120]], delay: [0,0,0,0,0,20], gripperAngle: 0.0}" --once


// Смотрит на куб
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.4775620698928833, -1.6089142004596155, 1.1463332176208496, -1.400895897542135, -1.5800517241107386, 0.00013182648399379104]], delay: [0,0,0,0,0,20], gripperAngle: 0.0}" --once



// схват куба 
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.0789810419082642, -0.5552123228656214, 1.9236936569213867, -3.0894487539874476, -1.5335939566241663, 0.0014620755100622773]], delay: [0,0,0,0,0,20], gripperAngle: 0.0}" --once


// скинуть куб
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.4775620698928833, -1.3088954130755823, 1.4463391304016113, -2.300877873097555, -1.5800159613238733, 0.00014381069922819734]], delay: [0,0,0,0,0,20], gripperAngle: 0.0}" --once






### 1. Подключение камеры

Чтобы работал просмотр с камер, необходимо:

1) На роботе запустить publisher а:
<pre><code>cd /home/administrator/rubleva/ur5_husky_api
catkin_make
source devel/setup.bash
roslaunch camera_pub camera.launch</code></pre>




### 2. Запуск ноды для гриппера

1) На роботе запустить publisher:
cd /home/administrator/rubleva/ur5_husky_api
catkin_make
source devel/setup.bash
roslaunch gripper_move gripper.launch

Минимальное положение гриппера - 0, максимальное - 0.085

rosservice call gripper_move "angle: 0.04"


### 3. Управление манипулятором






### 4. Получение информации о джоинтах



