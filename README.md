jaguar4x4wheel
==============

## Build from source

*ROS must be installed*

### Install wstool and catkin-tools

```bash
sudo apt-get install python-wstool python-catkin-tools
```

### Clone repo in catkin workspace

```bash
cd ~/catkin_ws/src/
git clone https://github.com/vincentrou/jaguar4x4wheel.git
```

### Download missing ros dependency

```bash
cd jaguar4x4wheel
rosdep install --from-paths . -i -y
```

### Download other repos with wstool

```bash
wstool init . jaguar4x4wheel/.rosinstall
```

### Build you workspace

```bash
catkin build
```

## Launch robot in simulation
`roslaunch jaguar4x4wheel_gazebo jaguar4x4wheel_empty_world.launch`

`roslaunch jaguar4x4wheel_control control.launch`

## Launch robot in real

`roslaunch jaguar4x4wheel_base base.launch`

