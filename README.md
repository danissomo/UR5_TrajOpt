# Задача открывания двери роботом-манипулятором UR5 при помощи алгоритма TrajOpt

<p>1. Склонировать репозиторий <code>git clone https://github.com/allicen/trajopt_ur5</code>.</p>

<p> 2. Перейти в папку проекта <code>cd trajopt_ur5</code>.</p>

<p> 3. Собрать окружение для робота в docker-контейнер: <code>sudo docker build -t trajopt-img . --network=host --build-arg from=ubuntu:20.04</code></p>

<p><br /></p>

<p> 4. Запустить робота</p>

<p><u>Новое окно терминала:</u></p>

<p>Запустить docker-контейнер с окружением для робота: <code>sudo ./scripts/docker/run_armbot_docker.sh</code></p>

<p>Перейти в рабочую директорию <code>cd workspace</code></p>

<p>Собрать проект <code>catkin build</code></p>

<p>Прописать пути <code>source devel/setup.bash</code></p>

<p>Запустить окружение робота <code>roslaunch ur5_single_arm_manipulation load_scene.launch</code> (загрузка с параметрами: <code>roslaunch ur5_single_arm_manipulation load_scene.launch x:=2 y:=2 z:=0</code>).</p>

<p>
    <img src="media/pic1.png" alt="Demo" style="max-width: 100%;" />
</p>

<p><br /></p>

<p>5. Запустить управление роботом</p>

<p><u>Новое окно терминала:</u></p>

<p>Зайти в docker-контейнер <code>sudo docker exec -ti trajopt bash</code></p>

<p>Перейти в рабочую директорию <code>cd workspace</code></p>

<p>Прописать пути <code>source devel/setup.bash</code></p>

<p>Управление движениями: <code>roslaunch ur5_single_arm_manipulation move.launch pipeline:=trajopt</code></p>

<p><br /></p>

<p>6. Запустить команду открытия двери</p>

<p><u>Новое окно терминала:</u></p>

<p>Зайти в docker-контейнер <code>sudo docker exec -ti trajopt bash</code></p>

<p>Перейти в рабочую директорию <code>cd workspace</code></p>

<p>Прописать пути <code>source devel/setup.bash</code></p>

<p>Управление движениями: <code>rosrun ur5_single_arm_manipulation open</code></p>

<p><br /></p>

<hr />

<p><b>Другие команды:</b></p>

<p>Отправить команду с координатой для схвата робота: <code>rosrun ur5_single_arm_manipulation position _param:="1 2 3"</code></p>

<p>Вернуть робота в позу по умолчанию <code>rosrun ur5_single_arm_manipulation pose _param:="up"</code> (варианты поз: home, up, pickup)</p>

<p>Разомкнуть схват на определенный угол: <code>rosrun ur5_single_arm_manipulation gripper _param:="0.564"</code> (0 - открыт полностью)</p>

<p><br /></p>

<hr />

<p><b>Видео</b></p>

<p>
    <img src="media/video1.gif" alt="Start demo" style="max-width: 100%;">
</p>

<p>
    <img src="media/video2.gif" alt="Move" style="max-width: 100%;">
</p>
