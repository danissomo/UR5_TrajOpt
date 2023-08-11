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
- **src/ur5_husky_api** - пакеты, которые нужно запускать на самом роботе (опционально), подробнее см. <a href="https://github.com/allicen/UR5_TrajOpt/tree/main/src/ur5_husky_api">здесь</a>
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

#### 1. Собрать ur_rtde

Необходимо для управления роботом. Робот должен быть подключен через Ethernet.

<pre><code>cd /workspace/src/ur_rtde-v1.5.0
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make
sudo make install</code></pre>

Примените изменения <code>source ~/.bashrc</code>

#### 2. Настроить скрипты

- Дать права на запуск скриптов: <code>sudo chmod +x scripts/*sh</code>

При первом запуске скриптов нужно исправить ошибку перед запуском скриптов (преобразование окончаний строк из формата DOS в формат UNIX):

- Выполнить <code>sed -i -e 's/\r$//' "$ARMBOT_PATH/scripts/fix.sh"</code>
- Выполнить <code>./scripts/fix.sh</code>


## Как управлять роботом с помощью TrajOpt

**1. Склонировать репозиторий** <code>git clone --recurse-submodules https://github.com/allicen/trajopt_ur5</code>.

**2. Перейти в папку проекта** <code>cd trajopt_ur5</code>.

**3. Собрать окружение для робота** в docker-контейнер: <code>sudo docker build -t trajopt-img . --network=host --build-arg from=ubuntu:20.04</code>

**4. Запустить робота**

<u>Новое окно терминала:</u>

- Запустить docker-контейнер с окружением для робота: на 1 машине (на локальной) <code>sudo ./scripts/docker/run_armbot_docker.sh</code>, если планируется 2 машины (на локальной и на роботе): <code>sudo ./scripts/docker/run_armbot_docker.sh $ROS_MASTER_URI $ROS_IP</code>
- Перейти в рабочую директорию <code>cd workspace</code>
- Собрать проект <code>catkin build</code>
- Прописать пути <code>source devel/setup.bash</code>

**5. Запуск сцены**

Запуск trajopt без управления через UI <code>roslaunch ur5_husky_main run_ur5_husky_trajopt.launch ui_control:=false</code>

Запуск trajopt с UI <code>roslaunch ur5_husky_main run_ur5_husky_trajopt.launch</code>

**6. Управляйте роботом** из терминала или через UI.

Для управления из терминала:

<u>Новая вкладка терминала:</u>

- Зайти в docker-контейнер <code>sudo docker exec -ti trajopt bash</code>
- Прописать пути <code>source devel/setup.bash</code>
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

Зайти в docker-контейнер <code>sudo docker exec -ti trajopt bash</code>

Запустить тележку: <code>roslaunch ur5_husky_main robot_control.launch</code>

Запустить Freedrive <code>roslauch ur5_husky_main freedrive_node.launch</code>


#### Запуск просморта изображений с камер робота

1) На роботе запустить публикатора:
cd /home/administrator/rubleva/ur5_husky_api
catkin_make
source devel/setup.bash
roslaunch camera_pub camera.launch

2) В проекте:
roslaunch ur5_husky_camera camera.launch

#### Запуск ноды для гриппера

1) На роботе запустить publisher:
cd /home/administrator/rubleva/ur5_husky_api
catkin_make
source devel/setup.bash
roslaunch gripper_move gripper.launch

Минимальное положение гриппера - 0, максимальное - 0.085

2) В проекте:
rosservice call gripper_move "angle: 0.04"


## Решение проблем

**1) Ошибка с пакетом robotiq_ft_sensor** (втречается только на ноутбуке Dell)

Текст проблемы:

```resource not found: robotiq_ft_sensor```

Решение:
```
catkin clean robotiq_ft_sensor
catkin build robotiq_ft_sensor
```




