# mujoco_tutorials

## Installation
```bash
pip3 install rospkg
pip3 install mujoco
pip3 install opencv-contrib-python
pip3 install casadi
pip3 install gym
pip3 install torch
pip3 install stable-baselines3[extra]
pip3 install cvxopt
pip3 install -U scipy

git clone git@github.com:rohanpsingh/mujoco-python-viewer.git
cd mujoco-python-viewer
pip3 install -e .
```

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
wstool init .
# please edit .rosinstall as below
```

```yaml
- git:
    local-name: agent-system/lecture2023
    uri: git@github.com:agent-system/lecture2023.git
    version: main
- git:
    local-name: misc/rqt_virtual_joystick
    uri: git@github.com:haraduka/rqt_virtual_joystick.git
    version: noetic-devel
- git:
    local-name: misc/franka_ros
    uri: git@github.com:frankaemika/franka_ros.git
    version: develop
- git:
    local-name: misc/mujoco_menagerie
    uri: git@github.com:haraduka/mujoco_menagerie.git
    version: fix_for_agent_system
```

```bash
wstool update -t .
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -y -r --from-paths . --ignore-src
cd ../
catkin build
source devel/setup.bash
```

For ubuntu18.04, if you want to publish camera image, please install https://github.com/ros-perception/vision_opencv by Python3 as below,
```bash
mkdir -p ~/python3_ws/src
wstool init .
git clone -b $ROS_DISTRO https://github.com/ros-perception/vision_opencv.git
git clone https://github.com/ros/geometry2.git
rosdep install -y -r --from-paths . --ignore-src
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --install
cd ..
catkin build
source ~/python3_ws/devel/setup.bash --extend
```
