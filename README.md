
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

  
#  A simple ROS publisher & subscriber


A simple ROS publisher & subscriber package written in C++. Vallidated on Ubuntu 20.04 LTS and ROS Noetic.

## Steps to build the package

  Make a catkin workspace catkin_ws and run the following commands :
  

    cd <path_to_ws>/catkin_ws/src
    git clone https://github.com/llDev-Rootll/beginner_tutorials.git
    cd ../
    catkin_make
## Steps to run
In a terminal run :

    roscore
In another terminal run : 

    source devel/setup.bash
    rosrun beginner_tutorials talker
In another terminal run : 

    source devel/setup.bash 
    rosrun beginner_tutorials listener
    
## Steps to run using the launch file

In a terminal run :

    source devel/setup.bash
    roslaunch beginner_tutorials pub_sub.launch rate:=10
where rate is the command line argument for changing the rate of published messages

## Steps to run the service for changing the output string

With the talker and lisener running, in a terminal run :

     rosservice call /change_output "New Message"

## Running cpplint & cppcheck tests
Run the following command in the root directory to generate cpplint results in **results** folder
 
    sh run_cpplint.sh
Run the following command in the root directory to generate cppcheck results in **results** folder

    sh run_cppcheck.sh

  
<img alt="logs" src="assets/logs.png" width="75%" />

*Fig 1 :  Logs*