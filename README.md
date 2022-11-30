# Задача открывания двери роботом-манипулятором UR5 при помощи алгоритма TrajOpt

<p>1. Склонировать репозиторий <code>git clone https://github.com/allicen/trajopt_ur5</code>.</p>

<p> 2. Перейти в папку проекта <code>cd trajopt_ur5</code>.</p>

<p> 3. Собрать окружение для робота в docker-контейнер: <code>sudo docker build -t trajopt-img . --network=host --build-arg from=ubuntu:18.04</code></p>

<p><br /></p>

<p> 4. Запустить робота</p>

<p><b>1 терминал:</b></p>

<p>Запустить docker-контейнер с окружением для робота: <code>sudo ./scripts/docker/run_armbot_docker.sh</code></p>

<p>Перейти в рабочую директорию <code>cd workspace</code></p>

<p>Собрать проект <code>catkin_make</code> (если будут ошибки сборки, можно попробовать собрать пакеты изолированно <code>catkin_make_isolated</code>)</p>

<p>Прописать пути <code>source devel/setup.bash</code> (или <code>source devel_isolated/setup.bash</code>)</p>

<p>Запустить окружение робота <code>roslaunch ur5_single_arm_manipulation load_scene.launch</code></p>

<p><br /></p>

<p>5. Запустить управление роботом</p>

<p><b>2 терминал:</b></p>

<p>Зайти в docker-контейнер <code>sudo docker exec -ti trajopt bash</code></p>

<p>Перейти в рабочую директорию <code>cd workspace</code></p>

<p>Прописать пути <code>source devel/setup.bash</code></p></p>

<p>Управление движениями: <code>roslaunch ur5_single_arm_manipulation move.launch pipeline:=trajopt</code></p>

<p><b>Новый терминал:</b></p>

<p>Отправить команду с координатой: <code>rosrun ur5_single_arm_manipulation position _param:="1 2 3"</code> (для ориентации введите 4й символ)</p>

<p>Вернуть робота в позу по умолчанию <code>rosrun ur5_single_arm_manipulation pose _param:="up"</code> (варианты поз: home, up, pickup)</p>

<p>Разомкнуть схват на определенный угол: <code>rosrun ur5_single_arm_manipulation gripper _param:="0.564"</code> (0 - открыт полностью)</p>

<hr />

<p>Создать библиотеки:</p>
<p>
<pre><code>cd your catkin_ws_path/src/package</code>
<code>PKG_CONFIG_PATH=`rospack find gazebo`/gazebo/lib/pkgconfig cmake ../..</code></pre></p>