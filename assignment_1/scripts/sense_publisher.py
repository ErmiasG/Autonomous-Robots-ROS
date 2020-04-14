#! /usr/bin/env python

import time
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from pb_sense_hat.msg import SenseInputEvent
from sense_hat import SenseHat

class SensePublisher:

    def __init__(self):
        self.sense = SenseHat()
        self.sense.clear()
        self.pub_humidity = rospy.Publisher('sensehat/humidity', Float64, queue_size=10)
        self.pub_temperature = rospy.Publisher('sensehat/temperature', Float64, queue_size=10)
        self.pub_pressure = rospy.Publisher('sensehat/pressure', Float64, queue_size=10)
        self.pub_accelerometer = rospy.Publisher('sensehat/accelerometer', Vector3, queue_size=10)
        self.pub_gyroscope = rospy.Publisher('sensehat/gyroscope', Vector3, queue_size=10)
        self.pub_magnetometer = rospy.Publisher('sensehat/magnetometer', Vector3, queue_size=10)
        self.pub_compass = rospy.Publisher('sensehat/compass', Float64, queue_size=10)
        self.pub_stick = rospy.Publisher('sensehat/stick', SenseInputEvent, queue_size=10)

    def publish(self):
        self.pub_humidity.publish(self.sense.get_humidity())
        self.pub_temperature.publish(self.sense.get_temperature())
        self.pub_pressure.publish(self.sense.get_pressure())
        acceleration = self.sense.get_accelerometer_raw()
        self.pub_accelerometer.publish(Vector3(acceleration['x'], acceleration['y'], acceleration['z']))
        gyroscope = self.sense.get_gyroscope_raw()
        self.pub_gyroscope.publish(Vector3(gyroscope['x'], gyroscope['y'], gyroscope['z']))
        compass = self.sense.get_compass_raw()
        self.pub_magnetometer.publish(Vector3(compass['x'], compass['y'], compass['z']))
        self.pub_compass.publish(self.sense.get_compass())
        stickEvents = self.sense.stick.get_events()
        if len(stickEvents) > 0:
            event = SenseInputEvent(stickEvents[-1].timestamp, stickEvents[-1].direction, stickEvents[-1].action)
            self.pub_stick.publish(event)

    def turn_off(self):
        self.sense.clear()

    def run(self):
        rospy.init_node('sense_hat', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    try:
        s = SensePublisher()
        s.run()
    except rospy.ROSInterruptException:
        s.turn_off()
    