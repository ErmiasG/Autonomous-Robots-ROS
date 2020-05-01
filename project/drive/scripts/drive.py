#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Twist
import PicoBorgRev

class Drive:

    def __init__(self):
        sys.stdout = sys.stderr
        self.sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
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
        rospy.spin()

if __name__ == '__main__':
    try:
        d = Drive()
        rospy.on_shutdown(d.turn_off)
        d.run()
    except rospy.ROSInterruptException:
        d.turn_off()