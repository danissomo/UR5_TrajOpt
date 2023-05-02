# Перемещение робота-манипулятора UR5 при помощи алгоритма TrajOpt

1. Склонировать репозиторий <code>git clone https://github.com/allicen/trajopt_ur5</code>.

2. Перейти в папку проекта <code>cd trajopt_ur5</code>.

3. Собрать окружение для робота в docker-контейнер: <code>sudo docker build -t trajopt-img . --network=host --build-arg from=ubuntu:20.04</code>


4. Запустить робота

<u>Новое окно терминала:</u>

- Запустить docker-контейнер с окружением для робота: <code>sudo ./scripts/docker/run_armbot_docker.sh</code>
- Перейти в рабочую директорию <code>cd workspace</code>
- Собрать проект <code>catkin build</code>
- Прописать пути <code>source devel/setup.bash</code>

5. Запуск сцены

Запуск trajopt <code>roslaunch ur5_husky_main run_ur5_husky_trajopt.launch</code>

6. Следуйте командам в терминале или управляйте из UI.

<img src="media/image.png" />


#### Другие команды:

Зайти в docker-контейнер <code>sudo docker exec -ti trajopt bash</code>

Запись значений джоинтов на UR5 <code>roslaunch ur5_single_arm_manipulation ur5_set_joint_pos.launch</code>

Запустить тележку: <code>roslaunch ur5_husky_main robot_control.launch</code>

Запуск trajopt <code>roslaunch ur5_single_arm_manipulation move_robot_trajopt_ex.launch</code> (раннее)

Запуск rViz для подбора поз робота  <code>roslaunch ur5_single_arm_manipulation ur5.rviz.launch</code>

#### Собрать ur_rtde (опционально)
<pre><code>cd /workspace/src/ur_rtde/build
cmake ..
make
sudo make install</code></pre>
