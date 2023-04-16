# mujoco_tutorials

## Installation
```bash
pip3 install rospkg
pip3 install mujoco
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
    uri: git@github.com:deepmind/mujoco_menagerie.git
    version: main
```

```bash
wstool update -t .
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -y -r --from-paths . --ignore-src
cd ../
catkin build mujoco_tutorials
source devel/setup.bash
```

For ubuntu18.04, if you want to publish camera image, please install https://github.com/ros-perception/vision_opencv by Python3 as below,
```bash
mkdir -p ~/python3_ws/src
cd ~/python3_ws
git clone -b $ROS_DISTRO https://github.com/ros-perception/vision_opencv.git
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --install
catkin build cv_bridge
source /path/to/python3_ws/devel/setup.bash
```
