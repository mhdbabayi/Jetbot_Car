#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from jetbotcar.msg import jetRacerDriveMsg # float64 throttle, steering

def stopCallback(msg):
    print('stop stop stop')

def main():
    rospy.init_node('StopButton', anonymous=True)
    # key cmd subscriber
    keyTopic = rospy.get_param("/jetRacerDriveNode/keyboard_topic")
    rospy.Subscriber(keyTopic, String, stopCallback)


    # key cmd publisher
    driveTopic = rospy.get_param("/jetRacerDriveNode/jetracer_drive_topic")
    # drive_pub = rospy.Publisher(driveTopic,jetRacerDriveMsg, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()


