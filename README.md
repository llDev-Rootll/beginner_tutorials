[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

  
#  A simple ROS publisher & subscriber


A simple ROS publisher & subscriber package written in C++. Vallidated on Ubuntu 20.04 LTS and ROS Noetic.

## Steps to run

  Make a catkin workspace catkin_ws and run the following commands :
  

    cd <path_to_ws>/catkin_ws/src
    git clone https://github.com/llDev-Rootll/beginner_tutorials.git
    cd ../
    catkin_make

In a terminal run :

    roscore
In another terminal run : 

    source devel/setup.bash
    rosrun beginner_tutorials talker
In another terminal run : 

    source devel/setup.bash 
    rosrun beginner_tutorials listener
    


## Running cpplint & cppcheck tests
Run the following command in the root directory to generate cpplint results in **results** folder
 
    sh run_cpplint.sh
Run the following command in the root directory to generate cppcheck results in **results** folder

    sh run_cppcheck.sh

