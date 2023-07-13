## Здесь находятся пакеты, которые необходимо запускать на роботе

На роботе установлен ROS Melodic.

При создании новых пакетов для робота необходимо добавлять файл <code>CATKIN_IGNORE</code>, чтобы игнорировать их при сборке проекта.

После загрузки пакетов на робота удалить файлы <code>CATKIN_IGNORE</code>.

Путь до проекта на роботе: <code>/home/administrator/rubleva/ur5_husky_api</code>. Все пакеты из этой папки необходимо загрузить на робота и запустить их.

Перед запуском <code>ur5_info.launch</code> нужно поменять ip робота в файле или передавать ip аргументом <code>robot_ip:="127.0.0.1"</code>.

1 терминал
<code>cd /home/administrator/rubleva/ur5_husky_api</code>
<code>catkin_make</code>
<code>source devel/setup.bash</code>
<code>roslaunch ur5_info ur5_info.launch</code> или <code>roslaunch ur5_info ur5_info.launch robot_ip:="127.0.0.1"</code>

2 терминал 
Вызвать сервисы для управления гриппером или для управления манипулятором

а) Управление гриппером
<pre><code>cd /rubleva/ur5_husky_api
source devel/setup.bash
rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.0" --once</code></pre>

Минимальное положение гриппера - 0, максимальное - 0.085

б) Управление манипулятором
<pre><code>cd /rubleva/ur5_husky_api
source devel/setup.bash
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[]], delay: [], gripperAngle: 0.0}" --once</code></pre>

ВАЖНО!!!
position и delay - передавать массивы одинаковой длины

### Примеры команд

// схватить куб
<code>rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.04" --once</code>

// отпустить куб
<code>rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.085" --once</code>

// робот берет куб слева с пола
<code>rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[2.326473,-0.553581,1.686786,-2.687753,-1.561592,-0.000419]], delay: [0], gripperAngle: 0.0}" --once</code>

// робот в максимально сложенном положении (смотрит вниз)
<code>rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.606798,-3.091649,2.827192,-1.962667,-1.436540,-0.000551]], delay: [0], gripperAngle: 0.0}" --once</code>

// робот в сложенном состоянии смотрит вперед
<code>rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.595672, -2.871760, 2.799204, -3.072348,-1.581982, 0.000120]], delay: [0], gripperAngle: 0.0}" --once</code>

// робот смотрит на куб
<code>rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.4775620698928833, -1.6089142004596155, 1.1463332176208496, -1.400895897542135, -1.5800517241107386, 0.00013182648399379104]], delay: [0], gripperAngle: 0.0}" --once</code>

// робот у пола хвватает куб 
<code>rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.0789810419082642, -0.5552123228656214, 1.9236936569213867, -3.0894487539874476, -1.5335939566241663, 0.0014620755100622773]], delay: [0], gripperAngle: 0.0}" --once</code>

// робот в положении, из которого скидывает куб
<code>rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.4775620698928833, -1.3088954130755823, 1.4463391304016113, -2.300877873097555, -1.5800159613238733, 0.00014381069922819734]], delay: [0], gripperAngle: 0.0}" --once</code>

// передать 2 положения:
<code>rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.4775620698928833, -1.3088954130755823, 1.4463391304016113, -2.300877873097555, -1.5800159613238733, 0.00014381069922819734], position:[1.0789810419082642, -0.5552123228656214, 1.9236936569213867, -3.0894487539874476, -1.5335939566241663, 0.0014620755100622773]], delay: [0,0], gripperAngle: 0.0}" --once</code>


===================

### 1. Отдельное подключение камеры

Чтобы работал просмотр с камер, необходимо:

1) На роботе запустить publisher:
<pre><code>cd /home/administrator/rubleva/ur5_husky_api
catkin_make
source devel/setup.bash
roslaunch camera_pub camera.launch</code></pre>

2) Запустить на отдельном компьютере ноду для просмотра изображений с камер. 


### 2. Отдельное подключение гриппера

1) На роботе запустить publisher:
<pre><code>cd /home/administrator/rubleva/ur5_husky_api
catkin_make
source devel/setup.bash
roslaunch gripper_move gripper.launch</code></pre>

2) С другого компьютера отправлять данные для управления гриппером.
