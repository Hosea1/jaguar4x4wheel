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

### Download other repos with wstool

```bash
wstool init . jaguar4x4wheel/.rosinstall
```

### Build you workspace

```bash
catkin build
```
