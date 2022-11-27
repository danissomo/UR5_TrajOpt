# Задача открывания двери роботом-манипулятором UR5 при помощи алгоритма TrajOpt

1. Склонировать репозиторий <code>git clone https://github.com/allicen/trajopt_ur5</code>.

2. Перейти в папку проекта <code>cd trajopt_ur5</code>.

3. Собрать окружение для робота в docker-контейнер: <code>sudo docker build -t trajopt-img . --network=host --build-arg from=ubuntu:18.04</code>

4. Запустить робота

<b>1 терминал:</b>

Запустить docker-контейнер с окружением для робота: <code>sudo ./scripts/docker/run_armbot_docker.sh</code>

Перейти в рабочую директорию <code>cd workspace</code>

Собрать проект <code>catkin_make</code> (если будут ошибки сборки, можно попробовать собрать пакеты изолированно <code>catkin_make_isolated</code>)

Прописать пути <code>source devel/setup.bash</code>

Запустить окружение робота <code>roslaunch ur5_single_arm_manipulation run.launch pipeline:=trajopt</code>

5. Запустить управление роботом

<b>2 терминал:</b>

Зайти в docker-контейнер <code>sudo docker exec -ti trajopt bash</code>

Перейти в рабочую директорию <code>cd workspace</code>

Прописать пути <code>source devel/setup.bash</code>