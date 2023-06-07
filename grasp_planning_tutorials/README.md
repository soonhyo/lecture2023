# grasp_planning_tutorials

Tutorials for grasp planning with graspit and moveit.

## How to install

### (Recommended) Use VirtualBox image

- Install VirtualBox to your PC
  - Ubuntu:
    ```bash
    sudo apt install virtualbox
    ```
  - Windows:
    Download platform packages for Windows from https://www.virtualbox.org/wiki/Downloads and install it
- Download the image from (TBD)
- Launch VirtualBox and import the image
  - If you face an error "", you have to enable virtualization on your PC's BIOS
    - https://www.tekwind.co.jp/ASU/faq/entry_134.php
    - https://support.hp.com/jp-ja/document/ish_5637156-5698290-16
    - https://support.lenovo.com/jp/ja/solutions/ht500006-how-to-enable-virtualization-technology-on-lenovo-computers
- Start the image
  - If you face a kernel panic, disable ...

### (Not recommended) Install directly

#### Ubuntu 18.04 (with old but safe graspit)

```bash
mkdir -p ~/ros/ws_grasp_planning/src
cd ~/ros/ws_grasp_planning/src
wstool init . https://raw.githubusercontent.com/agent-system/lecture2023/main/grasp_planning_tutorials/melodic.rosinstall
rosdep install -y -r --ignore-src --from-paths graspit-ros graspit_interface graspit_commander lecture2023/grasp_planning_tutorials
cd ..
catkin build graspit
catkin build grasp_planning_tutorials
source ~/ros/ws_grasp_planning/devel/setup.bash  # Do this every time you open a new terminal
# Or you can write this in .bashrc to omit this:
# echo "source ~/ros/ws_grasp_planning/devel/setup.bash" >> ~/.bashrc
# Be careful because this may affect other software
rosrun grasp_planning_tutorials install_graspit_data
```

#### Ubuntu 18.04 (with the latest graspit)

```bash
# Install graspit
sudo apt install ...
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
catkin build grasp_planning_tutorials
source ~/ros/ws_grasp_planning/devel/setup.bash  # Do this every time you open a new terminal
# Or you can write this in .bashrc to omit this:
# echo "source ~/ros/ws_grasp_planning/devel/setup.bash" >> ~/.bashrc
# Be careful because this may affect other software
rosrun grasp_planning_tutorials install_graspit_data
```

#### (Not tested) Ubuntu 20.04 (with the latest graspit)

```bash
# Install graspit
sudo apt install ...
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
catkin build grasp_planning_tutorials
source ~/ros/ws_grasp_planning/devel/setup.bash  # Do this every time you open a new terminal
# Or you can write this in .bashrc to omit this:
# echo "source ~/ros/ws_grasp_planning/devel/setup.bash" >> ~/.bashrc
# Be careful because this may affect other software
rosrun grasp_planning_tutorials install_graspit_data
```

## How to use

```bash
roslaunch grasp_planning_tutorials fetch_pick_and_place.launch
# Wait until robot motion in gazebo finishes
rosrun grasp_planning_tutorials fetch_pick_and_place.py
```
