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
FRAME_ID = "odom" # 
CHILD_FRAME_ID = "base_link" # child_frame_id

class RobotPose:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.vx = 0
        self.vy = 0
        self.vtheta = 0

    def __str__(self):
        return str({'x': self.x, 'y': self.y, 'theta': self.theta,
                    'vx': self.vx, 'vy': self.vy, 'vtheta': self.vtheta})

class Drive:

    def __init__(self):
        sys.stdout = sys.stderr
        self.pose = RobotPose()
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

    def get_travel(self):
        power1 = -self.PBR.GetMotor1()
        power2 = self.PBR.GetMotor2()
        rotation1 = power1*RPM_MAX
        rotation2 = power2*RPM_MAX
        leftTravel = rotation1*WHEEL_CIRCUMFERENCE/60
        rightTravel = rotation2*WHEEL_CIRCUMFERENCE/60
        return leftTravel, rightTravel

    def update_pose(self, current_time):
        leftTravel, rightTravel = self.get_travel()
        deltaTime = (current_time - self.last_time).to_sec()

        deltaTravel = (rightTravel + leftTravel) / 2
        deltaTheta = (rightTravel - leftTravel) / AXLE_LENGTH

        if rightTravel == leftTravel:
            deltaX = leftTravel*cos(self.pose.theta)
            deltaY = leftTravel*sin(self.pose.theta)
        else:
            radius = deltaTravel / deltaTheta

            # Find the instantaneous center of curvature (ICC).
            iccX = self.pose.x - radius*sin(self.pose.theta)
            iccY = self.pose.y + radius*cos(self.pose.theta)

            deltaX = cos(deltaTheta)*(self.pose.x - iccX) \
                - sin(deltaTheta)*(self.pose.y - iccY) \
                + iccX - self.pose.x

            deltaY = sin(deltaTheta)*(self.pose.x - iccX) \
                + cos(deltaTheta)*(self.pose.y - iccY) \
                + iccY - self.pose.y

        self.pose.x += deltaX
        self.pose.y += deltaY
        self.pose.theta = (self.pose.theta + deltaTheta) % (2*pi)
        self.pose.vx = deltaTravel / deltaTime if deltaTime > 0 else 0.
        self.pose.vy = 0
        self.pose.vtheta = deltaTheta / deltaTime if deltaTime > 0 else 0.

        self.last_time = current_time

    def publish_odom(self):
        current_time = rospy.Time.now()
        
        self.update_pose(current_time)
        print("pose=", str(self.pose))
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.pose.theta)

        self.odom_broadcaster.sendTransform(
            (self.pose.x, self.pose.y, 0.),
            (odom_quat[0], odom_quat[1], odom_quat[2], odom_quat[3]),
            current_time,
            CHILD_FRAME_ID,
            FRAME_ID
        )

        # publish the odometry msg
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = FRAME_ID
        odom.child_frame_id = CHILD_FRAME_ID
        odom.pose.pose.position.x = self.pose.x
        odom.pose.pose.position.y = self.pose.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]
        odom.twist.twist.linear.x = self.pose.vx
        odom.twist.twist.linear.y = self.pose.vy
        odom.twist.twist.angular.z = self.pose.vtheta

        # publish the message
        self.pub_odom.publish(odom)

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
        self.last_time = rospy.Time.now()
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            self.publish_odom()
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