## Здесь находятся пакеты, которые необходимо запускать на роботе

При создании новых пакетов для робота необходимо добавлять файл <code>CATKIN_IGNORE</code>, чтобы игнорировать их при сборке проекта.

Путь до пакета на роботе: <code>/home/administrator/rubleva/ur5_husky_api</code>.

### 1. Подключение камеры

Чтобы работал просмотр с камер, необходимо:

1) На роботе запустить publisher а:
<pre><code>cd /home/administrator/rubleva/ur5_husky_api
catkin_make
source devel/setup.bash
roslaunch camera_pub camera.launch</code></pre>

