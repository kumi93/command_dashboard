command_dashboard
====

GUI application which publish desired pressures of Upper-Limb-Robot and 2DOF arm 

<img src="https://raw.github.com/wiki/kumi93/command_dashboard/images/Command_dashboard.png" width="650">


## Requirements
- arl_hw_msgs (https://github.com/arne48/arl_hw_msgs)
- arl_commons (https://github.com/arne48/arl_commons)

## Installation
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/kumi93/command_dashboard.git
```
and of course, 
```
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```


## Usage
```
$ rosrun command_dashboard command_dashboard.py 
```
For more information, please read help page in the application.
