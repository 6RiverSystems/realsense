#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Joy2Commands(object):

    ##############################################################################
    # Callback for the joystick input
    def callbackJoy(self, joy):

        horizontal = joy.axes[0]
        vertical = joy.axes[1]
        
        stopVelocity = joy.buttons[1]

        if stopVelocity:
            self.sendStop()
        else:
            if abs(horizontal) > self.JOYSTICK_THRESHOLD:
                self.sendRotate(1)
    
            if abs(vertical) > self.JOYSTICK_THRESHOLD:
                self.sendForward(10)

    ##############################################################################
    # Initialize the node properties
    def __init__(self):

        self.JOYSTICK_THRESHOLD = 0.1
        self.UPDATE_RATE = 100
        
        self.pubCmdLL = rospy.Publisher('/cmd_ll', String, queue_size=1)
        self.subJoy = rospy.Subscriber('joy', Joy, self.callbackJoy, queue_size=1)

        self.rate = rospy.Rate(self.UPDATE_RATE)

    ##############################################################################
    def printUsage(self):

        msg = """
Joystick to Commands Node                                    v 1.0
------------------------------------------------------------------
Lever             Move the robot

Button 2          STOP
------------------------------------------------------------------
        """
        
        print(msg)

    ##############################################################################
    # Execute the body of the driver
    def run(self):

        try:
            self.init()
            self.printUsage()

            while not rospy.is_shutdown():
                self.rate.sleep()
        
        except rospy.exceptions.ROSInterruptException:
            pass

    ##############################################################################
    # Send the array of commands
    def sendCommands(self, commands):

        message = ';'.join(commands)
        self.pubCmdLL.publish(message)

    ##############################################################################
    # Send the distance command
    def sendForward(self, distance):

        commands = [];
        commands.append("distance %.3f" % distance)
        self.sendCommands(commands)

    ##############################################################################
    # Publish the rotate command
    def sendRotate(self, angle):

        commands = [];
        commands.append("rotate %.3f" % angle)
        self.sendCommands(commands)

if __name__ == '__main__':
    rospy.init_node('Joy2Commands')
    j2c = Joy2Commands()
    j2c.run()

