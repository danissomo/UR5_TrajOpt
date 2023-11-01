# Перемещение робота-манипулятора UR5 при помощи алгоритма TrajOpt

Краткое описание проекта:

```

├── data
├── scripts
└── src
    ├── libs
    ├── pkgs_husky
    ├── pkgs_ur5
    ├── rosbag
    ├── settings_custom_lib
    ├── ur5_husky_api
    ├── ur5_husky_camera
    ├── ur5_husky_main
    └── ur_rtde-v1.5.0

```

- **data** - конфиги для управления роботом
- **scripts** - bash-скрипты
- **src/libs** - библиотеки <a href="https://github.com/tesseract-robotics">Tesseract-Robotics</a> (реализация алгоритма TrajOpt)
- **src/pkgs_husky** - пакеты для мобильной платформы Husky
- **src/pkgs_ur5** - пакеты для манипулятора Universal Robots (UR5) и гриппера Robotiq
- **src/rosbag** - пакеты для обработки росбэгов
- **src/settings_custom_lib** - пакет для добавления конфигов из data/settings.txt в другие пакеты
- **src/ur5_husky_camera** - пакет для получения изображения с камеры робота (Realsence и Zed), для получения изображений необходимо запускать соответствующий пакет на роботе (src/ur5_husky_api)
- **src/ur5_husky_main** - пакет с реализацией управления роботом с помощью алгоритма TrajOpt
- **src/ur_rtde-v1.5.0** - пакет с библиотекой ur_rtde (точно такая же версия используется на роботе).

<img src="media/image.png" />

Что реализовано в проекте:
- Получение положения джоинтов с робота
- Отправка положения джоинтов на робота
- Добавление, удаление, перемещение, поворот препятствий (препятствия добавляются из мешей - файлы .obj или .stl и можно добавить простой куб)
- Построение траектории с помощью TrajOpt (необходимо указать начальное положение или взять с робота, конечное положение и при желании промежуточные положения)
- Управление гриппером (размыкание на определенный угол)

Количество промежуточных положений может быть любым.

Количество препятствий может быть любым.

Варианты управления проектом:
1. Через web-интерфейс (управление положениями робота, препятствиями из мешей и построением траекторий); web-интерфейс в текущий проект не входит, jar-файл предоставлю по запросу (используется MySQL или могу собрать под встроенную в Spring Boot <a href="https://ru.wikipedia.org/wiki/H2">БД H2</a>);
2. Через терминал с использованием конфигов (data/settings.txt) - ограниченное управление (нет возможности добавлять разные виды препятствий, нет возможности добавлять промежуточные положения для манипулятора);
3. Через терминал с отправкой отдельных команд через сервисы и топики (пример команд приведены ниже).

## Предварительная подготовка

### 1. Склонировать репозиторий 

```git clone --recurse-submodules https://github.com/allicen/trajopt_ur5```

### 2. Настроить скрипты

- Дать права на запуск скриптов: ```sudo chmod +x ./trajopt_ur5/scripts/*sh```

При первом запуске скриптов нужно исправить ошибку перед запуском скриптов (преобразование окончаний строк из формата DOS в формат UNIX):

- Выполнить ```sed -i -e 's/\r$//' "./trajopt_ur5/scripts/fix.sh"```
- Выполнить ```./trajopt_ur5/scripts/fix.sh```

### 3. Закинуть пакеты на робота из проекта <a href="https://github.com/allicen/ur5_husky_api">ur5_husky_api</a>

В этом проекте содержатся пакеты, которые необходимо (или желательно) запускать при работе с реальным роботом. 

Следовать инструкциям в файле ```ur5_husky_api/readme.md```.

## Настройка окружения

**1. Перейти в папку проекта** ```cd trajopt_ur5```.

**2. Собрать окружение для робота** в docker-контейнер: ```sudo docker build -t trajopt-img . --network=host --build-arg from=ubuntu:20.04```

**3. Запустить окружение для робота**

**3.a. Реальный робот UR5**

Установите переменные среды.

Откройте файл ```.bashrc``` (каталог /home)

```
nano ~/.bashrc
```
и добавьте в него переменые:

```
export ROS_MASTER_URI=http://192.168.131.1:11311
export ROS_IP=192.168.131.16
```
Укажите свои IP-адреса робота (ROS_MASTER_URI - адрес платформы Husky, ROS_IP - адрес ноутбука, с которого будете управлять роботом).

Примените изменения ```source ~/.bashrc```.

- Запустить docker-контейнер с окружением для робота ```sudo ./scripts/docker/run_armbot_docker.sh $ROS_MASTER_URI $ROS_IP```


**3.б. Симулятор URSim**

<u>Развернуть URSim можно по инструкции</u>: https://hub.docker.com/r/universalrobots/ursim_cb3

```
docker pull universalrobots/ursim_cb3
docker run --rm -it universalrobots/ursim_cb3
```
**robot_ip указывать как 172.17.0.2**

- Запустить docker-контейнер с окружением для робота ```sudo ./scripts/docker/run_armbot_docker.sh```

**4. Собрать ur_rtde**

Необходимо для управления роботом. Робот должен быть подключен через Ethernet.

- Зайти в docker-контейнер в отдельном окне терминала ```sudo docker exec -ti trajopt bash```
- Собрать ur_rtde:

```
cd /workspace/src/ur_rtde-v1.5.0
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make
sudo make install
```

Закрыть окно терминала. 

**5. Собрать проект**

Вернуться в первое окно терминала.

- Перейти в рабочую директорию ```cd workspace```
- Собрать проект ```catkin build```
- Прописать пути ```source devel/setup.bash```


**6. Запуск сцены** 

Файл с разными настройками робота находится по адресу ```data/settings.txt```.

При необходимости замените настройки в нем. Основной параметр в файле, который нужнго проверить перед стартом, - это ```robot_ip = ***``` (здесь пропишите актуальный адрес робота UR5 или симулятора URSim! В противном случае оставьте значение 127.0.0.1).

6.а. Запуск основной сцены ```roslaunch ur5_husky_main run_ur5_husky_trajopt.launch robot_ip:=192.168.131.40```

В команду выше добавьте необходимые параметры (пример см. выше с командой robot_ip):

- ```ui_control:=false``` (для отключения запуска с GUI)
- ```use_robot:=true``` (если запускаете совместно с UR5 или URSim)

6.б.* Отдельно можно запустить ноду с получением информации о роботе ```roslaunch ur5_husky_main ur5_state.launch``` (запускать только если подключен URSim или UR5)

**7. Управляйте роботом** по командам терминала или через UI.

Для управления из терминала:

<u>Новая вкладка терминала:</u>

- Зайти в docker-контейнер ```sudo docker exec -ti trajopt bash```
- Перейти в рабочую директорию ```cd workspace```
- Прописать пути ```source devel/setup.bash```
- Ввести команду из списка ниже.

### Список команд для управления

**1. Получение траектории робота-манипулятора после применения TrajOpt**

**request**
```
rosservice call /calculate_robot_rajectory "{startPose: {name: ["ur5_shoulder_pan_joint", "ur5_shoulder_lift_joint", "ur5_elbow_joint", "ur5_wrist_1_joint", "ur5_wrist_2_joint", "ur5_wrist_3_joint"], position: [1.526473, -0.553581, 1.686786, -2.687753, -1.461592, -0.000419]}, finishPose: {name: ["ur5_shoulder_pan_joint", "ur5_shoulder_lift_joint", "ur5_elbow_joint", "ur5_wrist_1_joint", "ur5_wrist_2_joint", "ur5_wrist_3_joint"], position: [1.542354, -1.203353, 0.634284, -1.138470, -1.573804, -0.000371]}}"
```
где:
- startPose.position - начальное положение джоинтов
- finishPose.position - конечное положение джоинтов


**response success**
```
success: True
message: "Found valid solution"
trajectory:
  -
    name:
      - ur5_shoulder_pan_joint
      - ur5_shoulder_lift_joint
      - ur5_elbow_joint
      - ur5_wrist_1_joint
      - ur5_wrist_2_joint
      - ur5_wrist_3_joint
    position: [1.526473, -0.553581, 1.686786, -2.687753, -1.461592, -0.000419]
  -
    name:
      - ur5_shoulder_pan_joint
      - ur5_shoulder_lift_joint
      - ur5_elbow_joint
      - ur5_wrist_1_joint
      - ur5_wrist_2_joint
      - ur5_wrist_3_joint
    position: [1.5291198333333333, -0.6618763333333333, 1.5113690000000002, -2.4295391666666664, -1.480294, -0.000411]
  -
    name:
      - ur5_shoulder_pan_joint
      - ur5_shoulder_lift_joint
      - ur5_elbow_joint
      - ur5_wrist_1_joint
      - ur5_wrist_2_joint
      - ur5_wrist_3_joint
    position: [1.5317666666666667, -0.7701716666666666, 1.335952, -2.1713253333333333, -1.498996, -0.000403]
  -
    name:
      - ur5_shoulder_pan_joint
      - ur5_shoulder_lift_joint
      - ur5_elbow_joint
      - ur5_wrist_1_joint
      - ur5_wrist_2_joint
      - ur5_wrist_3_joint
    position: [1.5344134999999999, -0.8784669999999999, 1.1605349999999999, -1.9131115, -1.517698, -0.000395]
  -
    name:
      - ur5_shoulder_pan_joint
      - ur5_shoulder_lift_joint
      - ur5_elbow_joint
      - ur5_wrist_1_joint
      - ur5_wrist_2_joint
      - ur5_wrist_3_joint
    position: [1.5370603333333333, -0.9867623333333333, 0.9851179999999999, -1.6548976666666668, -1.5364, -0.00038700000000000003]
  -
    name:
      - ur5_shoulder_pan_joint
      - ur5_shoulder_lift_joint
      - ur5_elbow_joint
      - ur5_wrist_1_joint
      - ur5_wrist_2_joint
      - ur5_wrist_3_joint
    position: [1.5397071666666666, -1.0950576666666665, 0.809701, -1.3966838333333333, -1.555102, -0.000379]
  -
    name:
      - ur5_shoulder_pan_joint
      - ur5_shoulder_lift_joint
      - ur5_elbow_joint
      - ur5_wrist_1_joint
      - ur5_wrist_2_joint
      - ur5_wrist_3_joint
    position: [1.542354, -1.203353, 0.634284, -1.13847, -1.573804, -0.000371]
```

**responce failed**
```
success: False
message: "Failed to find valid solution"
trajectory: []
```


## Другие команды (не связанные с построением траекторий TrajOpt):

Зайти в docker-контейнер ```sudo docker exec -ti trajopt bash```

Запустить тележку: ```roslaunch ur5_husky_main robot_control.launch```

Запустить Freedrive ```roslauch ur5_husky_main freedrive_node.launch```


#### Запуск просморта изображений с камер робота

1) На роботе запустить публикатора:
```
cd /home/administrator/rubleva/ur5_husky_api
catkin_make
source devel/setup.bash
roslaunch camera_pub camera.launch
```

2) В проекте:
```roslaunch ur5_husky_camera camera.launch```
При проигрывании из росбэга: ```roslaunch ur5_husky_camera camera.launch rosbag:=true```

#### Запуск ноды для гриппера

1) На роботе запустить publisher:
```
cd /home/administrator/rubleva/ur5_husky_api
catkin_make
source devel/setup.bash
roslaunch gripper_move gripper.launch
```

Минимальное положение гриппера - 0, максимальное - 0.085

2) В проекте:
```rosservice call gripper_move "angle: 0.04"```


## Решение проблем

**1) Ошибка с пакетом robotiq_ft_sensor** (втречается только на ноутбуке Dell)

Текст проблемы:

```resource not found: robotiq_ft_sensor```

Решение:
```
catkin clean robotiq_ft_sensor
catkin build robotiq_ft_sensor
```


## Прочее

1. Собрать пакет без зависимостей ```catkin build ur5_husky_main --no-deps```

2. Очистить сборку ```catkin clean```

3. Из XACRO в URDF: ```rosrun xacro xacro src/ur5_husky_main/urdf/robot/trajopt.xacro > robot_ur5_2.urdf```


## Полезные ссылки

- Библиотека UR RTDE для управления роботом-манипулятором UR5: https://sdurobotics.gitlab.io/ur_rtde/api/api.html 
- Документация на библиотеки Tesseract (используется для алгоритма TrajOpt): https://tesseract-docs.readthedocs.io/en/latest/_source/intro/getting_started_doc.html
- Расчет кинематики и динамифи робота: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics 



