# RyCSV-Taller-2-Motion-Control
## Workshop #2 - RyCSV 2020-II

This is the ROS package solution for the Workshop #2 from the RyCSV course at UNAL

![KOBUKI RVIZ](https://i.ibb.co/37Vynkr/Rviz-Detail.png)

Robot simulation was accomplished using the [Kobuki desktop](http://wiki.ros.org/kobuki_desktop) package 

## Usage

**NOTE** 
Tested on:
* **Ubuntu 20.04** with **ROS Noetic** full desktop installation
* **Ubuntu 16.04** with **ROS Kinetic** full desktop installation

To run the simulation, use:

```bash
roslaunch rycsv_kobuki_motion_control gazebo_sim.launch
```
To toogle ON and OFF the world in Gazebo, add the world parameter (true by default):

```bash
roslaunch rycsv_kobuki_motion_control gazebo_sim.launch world:=false
```

## Team 
* **Jurgen Krejci** - [JurgenHK](https://github.com/JurgenHK) - _jhkrejcim@unal.edu.co
* **David Fonseca** - _daafonsecato@unal.edu.co_

