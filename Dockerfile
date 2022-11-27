ARG from

FROM ${from}

RUN apt-get update && apt-get install -y apt-utils \
                                         lsb-release \
                                         mesa-utils \
                                         gnupg2 \
                                         net-tools \
                                         build-essential \
                                         wget \
                                         unzip \
                                         curl \
                                         git \
                                         mc \
                                         vim

ENV TZ=Asia/Yekaterinburg
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && DEBIAN_FRONTEND=noninteractive \
        apt-get install -y ros-melodic-desktop-full \
            ros-melodic-moveit \
            gazebo9 \
            ros-melodic-gazebo-ros-pkgs \
            ros-melodic-ros-control \
            ros-melodic-gazebo-ros-control \
            ros-melodic-geographic-info \
            ros-melodic-teleop-twist-keyboard \
            ros-melodic-joy \
            ros-melodic-effort-controllers \
            ros-melodic-controller-manager \
            ros-melodic-tf2-sensor-msgs \
            ros-melodic-openslam-gmapping \
            libsdl-image1.2-dev \
            libsdl-dev \
            python-rosdep \
            python-rosinstall \
            python-rosinstall-generator \
            python-wstool \
            python-catkin-tools \
            libcanberra-gtk-module \
            libcanberra-gtk3-module \
            ros-melodic-pid \
            ros-melodic-rosserial-arduino \
            ros-melodic-rosserial \
            ros-melodic-moveit-planners-chomp \
            ros-melodic-moveit-visual-tools \
            ros-melodic-rosbridge-server \
            ros-melodic-visp* && \
    rosdep init && rosdep update && \
    echo "source /opt/ros/melodic/setup.bash"  >> ~/.bashrc && \
    echo "source /workspace/devel/setup.bash"  >> ~/.bashrc

RUN apt-get install -y libvisp-dev libvisp-doc
RUN apt-get install -y python-pip
RUN pip install pathlib statistics scipy

RUN apt-get install ros-melodic-joint-state-publisher-gui
RUN apt-get install ros-melodic-franka-description
RUN apt-get install ros-melodic*joint-trajectory-controller*