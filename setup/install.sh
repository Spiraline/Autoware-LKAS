# Install ROS melodic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full -y
source /opt/ros/melodic/setup.bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-rosdep -y
sudo rosdep init
rosdep update

echo "ROS Install Sucess"

# System Dependencies of Ubuntu 18.04 / ROS Melodic
sudo apt-get update -y
sudo apt install -y python-catkin-pkg python-rosdep ros-melodic-catkin
sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool
pip3 install -U setuptools

# Eigen Build
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz
mkdir eigen
tar --strip-components=1 -xzvf eigen-3.3.7.tar.gz -C eigen
cd eigen
mkdir build
cd build
cmake ..
make
sudo make install
cd ../..
rm -rf eigen eigen-3.3.7.tar.gz

sudo rm /usr/lib/cmake/eigen3/*
sudo cp /usr/local/share/eigen3/cmake/* /usr/lib/cmake/eigen3

# Install rosdep
cd autoware.ai
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro melodic
cd ..

# Resolve OpenCV version issue
sudo apt-get install libopencv3.2 -y
sudo cp setup/cv_bridgeConfig.cmake /opt/ros/melodic/share/cv_bridge/cmake
sudo cp setup/image_geometryConfig.cmake /opt/ros/melodic/share/image_geometry/cmake
sudo cp setup/grid_map_cvConfig.cmake /opt/ros/melodic/share/grid_map_cv/cmake

# Autoware Build
cd autoware.ai
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ..
ln -s autoware.ai ~/autoware.ai
source ~/autoware.ai/install/setup.bash

echo "Autoware Build Success"

# Build spiraline_ws
sudo apt-get install ros-melodic-ackermann-msgs ros-melodic-serial ros-melodic-veldoyne ros-melodic-velodyne-driver -y
cd spiraline_ws/src
catkin_init_workspace
cd ..
catkin_make
cd ..
ln -s spiraline_ws ~/spiraline_ws
source ~/spiraline_ws/devel/setup.bash

echo "spiraline_ws Build Success"