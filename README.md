# Cranfield Jetbot 
This repository contains code that can run both the RVIZ simulator and the robot. 

The simulator original code is taken from:

https://github.com/f1tenth-dev/simulator

The original robot code is taken from

https://github.com/dusty-nv/jetbot_ros

The instructions for building and running the code can be found here:

https://canvas.cranfield.ac.uk/courses/7192/pages/jetbot-running-the-robot

 # Jetracer

to use on the Jetracer car: 
make sure you have installed the python 3 version of ROS packages and the Adafruit library: 

```
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
pip3 install adafruit-circuitpython-servokit  
pip3 install Adafruit-MotorHAT
```
swith to the jetRacer branch 
build the code in the workspace root directory : 
```
cd <workspace directory>
catkin_make
```
source setup.bash file (add it to .bashrc for automatic sourcing in the future)

launch the jetRacer.launch file with the run_the_robot argument set to "true":
```
$roslaunch jetbotcar jetRacer.launch run_the_robot:="true"
```



