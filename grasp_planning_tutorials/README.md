# grasp_planning_tutorials

Tutorials for grasp planning and execution with graspit, moveit, and gazebo.

## How to install

### (Recommended) Use VirtualBox image

- Install VirtualBox to your PC
  - Ubuntu:
    ```bash
    sudo apt install virtualbox
    ```
  - Windows:
    Download platform packages for Windows from https://www.virtualbox.org/wiki/Downloads and install it
- Download the image from https://drive.google.com/file/d/1NeoblfYeY-UPyACOcD16CZTK3uvjPIl0/view?usp=drive_link
- Launch VirtualBox and import the image
  - If you face `VT-x is not available (VERR_VMX_NO_VMX).` error, you have to enable virtualization on your PC's BIOS
    - https://www.tekwind.co.jp/ASU/faq/entry_134.php
    - https://support.hp.com/jp-ja/document/ish_5637156-5698290-16
    - https://support.lenovo.com/jp/ja/solutions/ht500006-how-to-enable-virtualization-technology-on-lenovo-computers
- Start the image
  - Password: ubuntu
  - If you face a kernel panic, right-click the image -> `Setting` -> `Storage` -> `Controller: SATA` -> uncheck `Use Host I/O Cache`
    - This will make the virtual machine slow

### (Not recommended) Install directly

#### Ubuntu 18.04, ROS Melodic (with old but safe graspit)

1. [Install ROS Melodic Desktop-Full, setup environment, and install dependencies for building packages](https://wiki.ros.org/melodic/Installation/Ubuntu)
2. Setup a workspace for grasp_planning_tutorials:
   ```bash
   mkdir -p ~/ros/ws_grasp_planning/src
   cd ~/ros/ws_grasp_planning/src
   wstool init . https://raw.githubusercontent.com/agent-system/lecture2023/main/grasp_planning_tutorials/melodic.rosinstall
   rosdep install -y -r --ignore-src --from-paths graspit-ros graspit_interface graspit_commander lecture2023/grasp_planning_tutorials
   cd ..
   sudo apt install python-catkin-tools
   catkin build graspit
   catkin build grasp_planning_tutorials
   source ~/ros/ws_grasp_planning/devel/setup.bash  # Do this every time you open a new terminal
   # Or you can write this in .bashrc to omit this:
   # echo "source ~/ros/ws_grasp_planning/devel/setup.bash" >> ~/.bashrc
   # Be careful because this may affect other software
   rosrun grasp_planning_tutorials install_graspit_data  # Ignore permission errors if symbolic links are finally created
   ```

## How to use

### Grasp planning with a multi-fingered hand

```bash
roslaunch graspit_interface graspit_interface.launch
ipython
```
In ipython interpreter,
```python
from graspit_commander import GraspitCommander
GraspitCommander.clearWorld()
GraspitCommander.loadWorld('plannerMug')  # Or 'plannerP51'
GraspitCommander.planGrasps(max_steps=50000)
```

### Grasp planning and execution with a parallel gripper

```bash
roslaunch grasp_planning_tutorials fetch_pick_and_place.launch
# Wait until robot motion in gazebo finishes
rosrun grasp_planning_tutorials fetch_pick_and_place.py
```

## Other installation methods

Currently, the parallel gripper sample does not work with these installation methods due to the following error:  
https://github.com/graspit-simulator/graspit/issues/170

### Ubuntu 18.04, ROS Melodic (with the latest graspit)

1. [Install ROS Melodic Desktop-Full, setup environment, and install dependencies for building packages](https://wiki.ros.org/melodic/Installation/Ubuntu)
2. Setup a workspace for grasp_planning_tutorials:
   ```bash
   # Install graspit
   sudo apt install libqt4-dev libqt4-opengl-dev libqt4-sql-psql libcoin80-dev libsoqt4-dev libblas-dev liblapack-dev libqhull-dev libeigen3-dev  # https://graspit-simulator.github.io/build/html/installation_linux.html#dependencies
   cd ~
   git clone https://github.com/graspit-simulator/graspit.git -b 83a6d4c5e0ccbcda9fa59cea17afbe6f096df785
   cd graspit
   mkdir build
   cd build
   cmake ..
   make
   sudo make install
   export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && export GRASPIT=~/.graspit  # Do this every time you open a new terminal
   # Or you can write this in .bashrc to omit this:
   # echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && export GRASPIT=~/.graspit" >> ~/.bashrc
   # Be careful because this may affect other software

   # Install other packages
   mkdir -p ~/ros/ws_grasp_planning/src
   cd ~/ros/ws_grasp_planning/src
   wstool init . https://raw.githubusercontent.com/agent-system/lecture2023/main/grasp_planning_tutorials/melodic_latest_graspit.rosinstall
   rosdep install -y -r --ignore-src --from-paths graspit_interface graspit_commander lecture2023/grasp_planning_tutorials
   cd ..
   sudo apt install python-catkin-tools
   catkin build grasp_planning_tutorials
   source ~/ros/ws_grasp_planning/devel/setup.bash  # Do this every time you open a new terminal
   # Or you can write this in .bashrc to omit this:
   # echo "source ~/ros/ws_grasp_planning/devel/setup.bash" >> ~/.bashrc
   # Be careful because this may affect other software
   rosrun grasp_planning_tutorials install_graspit_data  # Ignore permission errors if symbolic links are finally created
   ```

### (Not tested) Ubuntu 20.04, ROS Noetic (with the latest graspit)

1. [Install ROS Noetic Desktop-Full, setup environment, and install dependencies for building packages](https://wiki.ros.org/noetic/Installation/Ubuntu)
2. Setup a workspace for grasp_planning_tutorials:
   ```bash
   # Install graspit
   sudo apt install libcoin-dev libsoqt520-dev
   cd ~
   git clone https://github.com/graspit-simulator/graspit.git -b 83a6d4c5e0ccbcda9fa59cea17afbe6f096df785
   cd graspit
   mkdir build
   cd build
   cmake ..
   make
   sudo make install
   export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && export GRASPIT=~/.graspit  # Do this every time you open a new terminal
   # Or you can write this in .bashrc to omit this:
   # echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && export GRASPIT=~/.graspit" >> ~/.bashrc
   # Be careful because this may affect other software

   # Install other packages
   mkdir -p ~/ros/ws_grasp_planning/src
   cd ~/ros/ws_grasp_planning/src
   wstool init . https://raw.githubusercontent.com/agent-system/lecture2023/main/grasp_planning_tutorials/noetic_latest_graspit.rosinstall
   rosdep install -y -r --ignore-src --from-paths graspit_interface graspit_commander lecture2023/grasp_planning_tutorials
   cd ..
   sudo apt install python3-catkin-tools
   catkin build grasp_planning_tutorials
   source ~/ros/ws_grasp_planning/devel/setup.bash  # Do this every time you open a new terminal
   # Or you can write this in .bashrc to omit this:
   # echo "source ~/ros/ws_grasp_planning/devel/setup.bash" >> ~/.bashrc
   # Be careful because this may affect other software
   rosrun grasp_planning_tutorials install_graspit_data  # Ignore permission errors if symbolic links are finally created
   ```
