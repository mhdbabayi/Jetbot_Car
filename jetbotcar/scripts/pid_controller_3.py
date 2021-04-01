#!/usr/bin/env python
import rospy
from jetbotcar.msg import Jetdrivemsg # float64 left, right
from jetbotcar.msg import jetRacerDriveMsg # float64 throttle, steering
from jetbotcar.msg import jetErrorMsg # float64 lateralError
# from scipy import integrate

import rospy
import time

from std_msgs.msg import Float64

######
import sys
import math
import numpy as np




#PID CONTROL PARAMS
kp = 1
kd = 0.5
ki = 0
# lateralError = 0.0

Setpoint = 0.0 # follow the middle of the line is set as 0.0

Ts = 1
cumError = 0.0
lastError = 0.0

# INITIALISE THE THROTTLE AND STEERING CMDs
throttleCmd = 0.0
steeringCmd = 0.0
steeringGain = 0.0


def pidCallback(msg):

    global startTime
    global currentTime
    global Setpoint
    global Ts
    global cumError
    global lastError
    # global lateralError

    global kp, ki, kd
    global throttleCmd, steeringCmd

    publish = bool(True)

    lateralErrorCmd = msg.data # assign error messages from image processing to lateralError
   
    # Use kp, ki & kd to implement a PID controller for 

    # Time stamp

    currentTime = rospy.get_time()
    rospy.loginfo("Current time: %.2f", currentTime)

    elapsedTime = currentTime - startTime
    rospy.loginfo("Elapsed time: %.2f", elapsedTime)

    # Compute all the working error variables, careful with the sign convension
    error = lateralErrorCmd #lateral error is the error input from image processing
    rospy.loginfo("lateral error: %.2f", error)

    # cumError = cumError + error * elapsedTime
    # cumError = integrate.quad(error, 0, 0.1)
    rospy.loginfo("Cumulative Error: %.2f", cumError)

    rateError = (error - lastError)/elapsedTime
    # rospy.loginfo("rate error: %.2f", rateError)


    # PID output computation
    output = kp * error + ki * cumError + kd * rateError # need satruation
    rospy.loginfo("PID output: %.2f\n", output)


    # Remember some variables for next time
    lastError = error
    previousTime = currentTime

    # output conditions need to be modified
    ###################################
    if output > 0 and output < 5:
        throttleCmd = 0.5
        steeringCmd = output # probably 
        
    elif output > 5:
        throttleCmd = 0.5
        steeringCmd = 0.8 # saturation the max steering, condition need further calibration

    elif output > -5 and output < 0:
        throttleCmd = 0.5
        steeringCmd = output

    elif output < -5:
        throttleCmd = 0.5
        steeringCmd = -0.8 # saturation the max steering, condition need further calibration

    elif output == 0:
        throttleCmd = 0.5
        steeringCmd = output
    else:
        publish = bool(False)
    ###################################


    # if publish:
    #     self.publishJetracer(throttle, steering)


def main():
    global throttleCmd, steeringCmd
    global startTime
    global currentTime
    global previousTime
    # initialise pid_controller node
    rospy.init_node("pid_controller", anonymous=True)

 
    startTime = rospy.get_time() # startTime is constant now, stamped straight after init_node
    rospy.loginfo("startTime: %.2f", startTime)

    # subscribes to the lateral_error topic
    rospy.Subscriber("/lateral_error", Float64, pidCallback)

    # # Get topic name from the yaml file 
    # (just a topic title, doesn't include anything)
    driveTopic = rospy.get_param("/jetRacerDriveNode/jetracer_drive_topic")

    # jetRacerDriveMsg contains throttle and steering cmds
    drive_pub = rospy.Publisher(driveTopic,jetRacerDriveMsg, queue_size=10)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():

        # Write a publishing function
        drive_msg = jetRacerDriveMsg()

        drive_msg.throttle = throttleCmd
        drive_msg.steering = steeringCmd

        # drive_pub.publish(drive_msg) # from now don't publish drive_msg to LLC

        # print("Publishing control commands for throttle and steering")
        rate.sleep()

if __name__=='__main__':
	main()

