# Autoware with LKAS
Autoware with LKAS

## Publication


## Environment

- Ubuntu 18.04
- ROS Melodic
- OpenCV 4.x

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
### System Dependencies of Ubuntu 18.04 / ROS Melodic
```
sudo apt-get update
sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin
sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool
pip3 install -U setuptools
```

### Eigen build
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

### Install dependent packages
```
cd {$WORKSPACE_DIR}/autoware.ai
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

### Resolving OpenCV version issue
```
sudo apt-get install libopencv3.2 -y
```

You should change some code to use OpenCV 4.x.

If you have `/usr/local/opencv4` directory,

change `set(_include_dirs "include;/usr/include;/usr/include/opencv")`

to `set(_include_dirs "include;/usr/include;/usr/include/opencv4")`

in below three files (`sudo` required)
  - `/opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake`
  - `/opt/ros/melodic/share/image_geometry/cmake/image_geometryConfig.cmake`
  - `/opt/ros/melodic/share/grid_map_cv/cmake/grid_map_cvConfig.cmake`

### Autoware Build
```
# If you have CUDA
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build only some package
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select $(pakcage name)

# Build without some package
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip $(pakcage name)

# If you don't have CUDA
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## How to build package in spiraline_ws
* Install some dependency
```
sudo apt-get install ros-melodic-ackermann-msgs ros-melodic-serial ros-melodic-veldoyne ros-melodic-velodyne-driver -y
```

* Initialize ROS workspace
```
cd ${WORKSPACE_DIR}/spiraline_ws/src
catkin_init_workspace
```

* Build spiraline_ws packages
```
ln -s ${WORKSPACE_DIR}/autoware.ai ~/autoware.ai
echo "source ~/autoware.ai/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd ${WORKSPACE_DIR}/spiraline_ws
catkin_make
ln -s ${WORKSPACE_DIR}/spiraline_ws ~/spiraline_ws
echo "source ~/spiraline_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

```

## Additional Install for SVL
```
sudo apt-get install -y ros-melodic-rosbridge-server net-tools
pip3 install PyYAML rospkg matplotlib opencv-python
cd setup/svl
python3 -m pip install -r requirements.txt --user .
```

* Allow Firewall
```
sudo ufw allow 9090
```
