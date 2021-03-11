
from adafruit_servokit import ServoKit 
from jetbotcar.msg import Jetdrivemsg
import rospy
import time

class jetracer:

    i2c_address1 = 0x40
    i2c_address2 = 0x60
    steering_gain = 0.65
    steering_offset = 0
    steering_channel = 0
    throttle_gain = 0.8

    def __init__(self , *args, **kwargs):
        self.kit = ServoKit(channels=16, address=self.i2c_address1)
        self.motor= ServoKit(channels=16, address=self.i2c_address2)
        self.motor._pca.frequency = 1600
        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        rospy.init_node('jetracerLowLevel')
        #self.speedCommandTopic = rospy.get_param("/motorDriveNode/motorSpeedTopic")
        self.speedCommandTopic = 'diff_drive'
	#self.steeringCommandTopic = rospy.get_param("motorDriveNode/steeringCommandTopic")

        self.speedSub = rospy.Subscriber(self.speedCommandTopic, Jetdrivemsg, self.on_throttle)
        self.steeringSub = rospy.Subscriber(self.speedCommandTopic, Jetdrivemsg, self.on_steering)

    def on_steering(self, msg):
        steering_angle_cmd = msg.right
        self.steering_motor.throttle = steering_angle_cmd*self.steering_gain + self.steering_offset

    def on_throttle(self, msg):
        throttleValue = msg.right
        if throttleValue > 0:
            self.motor._pca.channels[0].duty_cycle = int(0xFFFF * (throttleValue * self.throttle_gain))
            self.motor._pca.channels[1].duty_cycle = 0xFFFF
            self.motor._pca.channels[2].duty_cycle = 0
            self.motor._pca.channels[3].duty_cycle = 0
            self.motor._pca.channels[4].duty_cycle = int(0xFFFF * (throttleValue * self.throttle_gain))
            self.motor._pca.channels[7].duty_cycle = int(0xFFFF * (throttleValue * self.throttle_gain))
            self.motor._pca.channels[6].duty_cycle = 0xFFFF
            self.motor._pca.channels[5].duty_cycle = 0
        else:
            self.motor._pca.channels[0].duty_cycle = int(-0xFFFF * (throttleValue * self.throttle_gain))
            self.motor._pca.channels[1].duty_cycle = 0
            self.motor._pca.channels[2].duty_cycle = 0xFFFF
            self.motor._pca.channels[3].duty_cycle = int(-0xFFFF * (throttleValue * self.throttle_gain))
            self.motor._pca.channels[4].duty_cycle = 0
            self.motor._pca.channels[7].duty_cycle = int(-0xFFFF * (throttleValue * self.throttle_gain))
            self.motor._pca.channels[6].duty_cycle = 0
            self.motor._pca.channels[5].duty_cycle = 0xFFFF
