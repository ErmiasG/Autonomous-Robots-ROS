#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import tf
import math
from math import sin, cos, pi
import PicoBorgRev

#Speed = (RPM (diameter * PI) / 60)
RPM_MAX = 60 # max rpm when power is 100%
WHEEL_CIRCUMFERENCE = 0.06*pi # (diameter * PI)
AXLE_LENGTH = 0.18 # length of the differential drive wheel axle

class Drive:

    def __init__(self):
        sys.stdout = sys.stderr
        self.last_time = rospy.Time.now()
        self.pose = Vector3(0.0, 0.0, 0.0)
        self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=50)
        self.sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.PBR = PicoBorgRev.PicoBorgRev()
        self.find_board()
        self.set_power()

    def set_power(self):
        # Power settings
        voltageIn = rospy.get_param('~voltage_in', 12.0) # Total battery voltage to the PicoBorg Reverse
        voltageOut = rospy.get_param('~voltage_out', 6.0) # Maximum motor voltage
        
        # Setup the power limits
        if voltageOut > voltageIn:
            self.maxPower = 1.0
        else:
            self.maxPower = voltageOut / float(voltageIn)

    def find_board(self):
        #self.PBR.i2cAddress = 0x44                  # Uncomment and change the value if you have changed the board address
        self.PBR.Init()
        if not self.PBR.foundChip:
            boards = PicoBorgRev.ScanForPicoBorgReverse()
            if len(boards) == 0:
                print ('No PicoBorg Reverse found, check you are attached :)')
            else:
                print ('No PicoBorg Reverse at address %02X, but we did find boards:' % (PBR.i2cAddress))
                for board in boards:
                    print ('    %02X (%d)' % (board, board))
                print ('If you need to change the I2C address change the setup line so it is correct, e.g.')
                print ('PBR.i2cAddress = 0x%02X' % (boards[0]))
            sys.exit()
        #self.PBR.SetEpoIgnore(True)                 # Uncomment to disable EPO latch, needed if you do not have a switch / jumper
        # Ensure the communications failsafe has been enabled!
        failsafe = False
        for i in range(5):
            self.PBR.SetCommsFailsafe(True)
            failsafe = self.PBR.GetCommsFailsafe()
            if failsafe:
                break
        if not failsafe:
            print ('Board %02X failed to report in failsafe mode!' % (PBR.i2cAddress))
            sys.exit()
        self.PBR.SetCommsFailsafe(False)
        self.PBR.ResetEpo()
        self.PBR.MotorsOff()

    def get_velocity(self):
        power1 = self.PBR.GetMotor1()
        power2 = self.PBR.GetMotor2()
        rpm1 = power1*RPM_MAX
        rpm2 = power2*RPM_MAX
        v_left = rpm1*WHEEL_CIRCUMFERENCE/60
        v_right = rpm2*WHEEL_CIRCUMFERENCE/60
        linear_velocity = (v_left + v_right)/2
        vth = (v_right - v_left)/AXLE_LENGTH
        return linear_velocity, -linear_velocity, vth

    def update_pos(self):
        current_time = rospy.Time.now()
        vx, vy, vth = self.get_velocity()
        
        th = self.pos.z
        dt = (current_time - last_time).to_sec()
        delta_x = (vx * cos(th) - vy * sin(th)) * dt
        delta_y = (vx * sin(th) + vy * cos(th)) * dt
        delta_th = vth * dt

        self.pose.x += delta_x
        self.pose.y += delta_y
        self.pose.z += delta_th

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        self.odom_broadcaster.sendTransform(
            (self.pose.x, self.pose.y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # publish the odometry msg
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.pos.x, self.pos.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        self.odom_pub.publish(odom)
        self.last_time = current_time

    def cmd_vel_callback(self, twist):
        upDown = twist.linear.x
        leftRight = twist.angular.z

        driveLeft = upDown
        driveRight = upDown
        if leftRight < -0.05:
            # Turning left
            driveLeft *= 1.0 + (2.0 * leftRight)
        elif leftRight > 0.05:
            # Turning right
            driveRight *= 1.0 - (2.0 * leftRight)

        self.PBR.SetMotor1(driveRight * self.maxPower)
        self.PBR.SetMotor2(-driveLeft * self.maxPower)

    def turn_off(self):
        #self.PBR.SetLed(False)
        self.PBR.MotorsOff()

    def run(self):
        rospy.init_node("drive")
        rospy.init_node("odometry")
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            self.update_pos()
            r.sleep()

        self.turn_off()
        #rospy.spin()

if __name__ == '__main__':
    try:
        d = Drive()
        rospy.on_shutdown(d.turn_off)
        d.run()
    except rospy.ROSInterruptException:
        d.turn_off()