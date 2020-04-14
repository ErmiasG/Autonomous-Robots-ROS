#! /usr/bin/env python

import time
import rospy
from std_msgs.msg import String
from assignment_1.msg import SenseInputEvent

class SenseSubscriber:

    def __init__(self):
        self.sub_humidity = rospy.Subscriber('sensehat/humidity', Float64, self.humidity_callback, queue_size=1)
        self.sub_temperature = rospy.Subscriber('sensehat/temperature', Float64, self.temperature_callback, queue_size=1)
        self.sub_pressure = rospy.Subscriber('sensehat/pressure', Float64, self.pressure_callback, queue_size=1)
        self.sub_accelerometer = rospy.Subscriber('sensehat/accelerometer', Vector3, self.accelerometer_callback, queue_size=1)
        self.sub_gyroscope = rospy.Subscriber('sensehat/gyroscope', Vector3, self.gyroscope_callback, queue_size=1)
        self.sub_magnetometer = rospy.Subscriber('sensehat/magnetometer', Vector3, self.magnetometer_callback, queue_size=1)
        self.sub_compass = rospy.Subscriber('sensehat/compass', Float64, self.compass_callback, queue_size=1)
        self.sub_stick = rospy.Subscriber('sensehat/stick', SenseInputEvent, self.stick_callback, queue_size=1)

    def humidity_callback(self, humidity):
        print("Humidity: ", humidity)

    def temperature_callback(self, temperature):
        print("Temperature: ", temperature)

    def pressure_callback(self, pressure):
        print("Pressure:", pressure)

    def accelerometer_callback(self, accelerometer):
        print("Accelerometer: ", accelerometer)

    def gyroscope_callback(self, gyroscope):
        print("Gyroscope: ", gyroscope)

    def magnetometer_callback(self, magnetometer):
        print("Magnetometer: ", magnetometer)

    def compass_callback(self, compass):
        print("Compass: ", compass)

    def stick_callback(self, stick):
        print("Joy stick: ", stick)


    def run(self):
        rospy.init_node("sense_hat_subscriber")
        rospy.spin()

if __name__ == '__main__':
    n = SenseListener()
    n.run()

