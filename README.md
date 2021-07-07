# hjw-Autoware
My Little Autoware

## Environment

- Ubuntu 18.04
- ROS Melodic
- CUDA 10.0

## How to install ROS melodic
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full -y
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep -y
sudo rosdep init
rosdep update
```

## How to build Autoware
* System Dependencies of Ubuntu 18.04 / ROS Melodic
```
sudo apt-get update
sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin
sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool
pip3 install -U setuptools
```

* Eigen build
```
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz
mkdir eigen
tar --strip-components=1 -xzvf eigen-3.3.7.tar.gz -C eigen
cd eigen
mkdir build
cd build
cmake ..
make
sudo make install
```

Older versions may already be installed. If `/usr/lib/cmake/eigen3/Eigen3Config.cmake` is older than 3.3.7 version, copy files in `/usr/local/share/eigen3/cmake` to `/usr/lib/cmake/eigen3`.
```
sudo rm /usr/lib/cmake/eigen3/*
sudo cp /usr/local/share/eigen3/cmake/* /usr/lib/cmake/eigen3
```

* Install dependent packages
```
cd {$WORKSPACE_DIR}/autoware.ai
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

* Autoware Build
```
# If you have CUDA
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# If you don't have CUDA
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Since Autoware recommend to use directory name 'autoware.ai', you should make soft link with autoware.ai to this repository
```
cd
ln -s ${WORKSPACE_DIR}/RUBIS-SelfDriving ~/autoware.ai
```

And it is recommned to add below sourcing command in your `~/.bashrc` file.
```
source ~/autoware.ai/install/setup.bash
```

## How to build package in rubis_ws

* Initialize ROS workspace
```
cd ${WORKSPACE_DIR}/rubis_ws/src
catkin_init_workspace
```

* Build rubis_ws packages
```
cd ${WORKSPACE_DIR}/rubis_ws
catkin_make
ln -s ${WORKSPACE_DIR}/rubis_ws ~/rubis_ws
source ~/rubis_ws/devel/setup.bash

```

## Create symoblic links
```
ln -s ${WORKSPACE_DIR}/autoware.ai ~/autoware.ai
ln -s ${WORKSPACE_DIR}/rubis_ws ~/rubis_ws
```

## How to launch LGSVL scrips
* Setup environments
```
cd ${WORKSPACE_DIR}/autoware.ai/autoware_files/lgsvl_file/scripts
pip3 install --user .
```

* Launch LGSVL scripts
```
sudo chomod 755 {TARGET_SCRIPTS}
./{TARGET_SCRIPTS}
```
