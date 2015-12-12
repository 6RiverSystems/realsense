#!/usr/bin/env python
import rospy
import math
import sys, os, time
import threading
import serial

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist

class BaseController(object):

    ENTITIES = {
        'tote0':        0,
        'tote1':        1,
        'tote2':        2,
        'tote3':        3,
        'tote4':        4,
        'tote5':        5,
        'tote6':        6,
        'tote7':        7,

        'action':     100,
        'pause':      101,

        'tail_left':  201,
        'tail_right': 202,
    }
    
    MODES = {
        'off':          0,
        'on':           1,
        
        'take':        10,
        'put':         11,
        
        'brake':      100,
        'turn':       101,
        'select':     102,
    }
    
    ##############################################################################
    # Initialize the node properties
    def __init__(self):

        self.rate = rospy.Rate(100)
        self.subCmdLL = rospy.Subscriber('/cmd_ll', String, self.cbCmdLL, queue_size=1)
        self.pubLLEvent = rospy.Publisher('/ll_event', String, queue_size=1)
        self.pubLLDebug = rospy.Publisher('/ll_debug', String, queue_size=1)

        self.usbCmd = serial.Serial()
        self.usbCmd.port = '/dev/malg'
        self.usbCmd.baudrate = 115200
        self.usbCmd.parity = 'N'
        self.usbCmd.rtscts = False
        self.usbCmd.xonxoff = False
        self.usbCmd.timeout = 1

        self.latestEvent = ''

    ##############################################################################
    def cbCmdLL(self, message):
        
        rospy.loginfo("Received command: [%s]" % message.data)

        # Split the message in separate commands
        commands = message.data.split(';')
        
        # For each command
        for portion in commands:
            
            # Separate the arguments
            command = portion.split(' ')
            
            # Interpret the command
            if command:
                success = False

                # stop
                if command[0] == 'stop':
                    success = self.sendCommandStop()
                
                # distance <distance [mm]>
                elif command[0] == 'distance':
                    d = int(command[1])
                    if d == 0:
                        success = self.sendCommandStop()
                    else:
                        success = self.sendCommandDistance(d)
                
                # rotate <angle [1/10deg]>
                elif command[0] == 'rotate':
                    angle = int(command[1])
                    success = self.sendCommandRotate(angle)
                
                # ui <entity> <mode>
                elif command[0] == 'ui':
                    entity = command[1]
                    mode = command[2]
                    success = self.sendCommandUI(entity, mode)
                
                # version
                elif command[0] == 'version':
                    success = self.sendCmdVersion()
    
                if success:
                    rospy.loginfo('Command accepted')
                else:
                    rospy.logerr('Command rejected')

        return

    ##############################################################################
    # Connect to the USB
    def connectUSB(self):

        self.usbCmd.close()

        retryRate = rospy.Rate(1)
        connected = False

        counter = 120
        while not connected and not rospy.is_shutdown():
            try:
                self.usbCmd.open()
                connected = True
                rospy.loginfo("Connected to USB [%s]" % self.usbCmd.port)
                
            except serial.SerialException as e:
                rospy.logerr("Could not open USB [%s]" % self.usbCmd.port)
                rospy.logerr(e)
                retryRate.sleep()

                counter -= 1
                if not counter:
                    return False

                pass
            
            except rospy.exceptions.ROSInterrutException:
                pass

        return True

    ##############################################################################
    # Publish the LL event
    def publishLLEvent(self, message):

        publishEvent = True
        event = ''

        # STOP event
        if message[0] == 's':
            event = 'status stopped'
 
        elif message[0] == 'a':
            event = 'move completed'

        elif message[0] == 'b':
            event = 'ui action'

        if self.latestEvent != event:
            rospy.loginfo("Publishing LL event: [%s]" % event)
            self.pubLLEvent.publish(event)
            self.latestEvent = event

    ##############################################################################
    # Run the node
    def run(self):

        rospy.loginfo('Base Controller Node - video version')

        if not self.connectUSB():
            exit(1)
        self.setupReadThread()

        try:
            while not rospy.is_shutdown():
                self.rate.sleep()
        except rospy.exceptions.ROSInterrutException:
            pass

    ##############################################################################
    def sendCommandWithInteger(self, command, arg):

        low = arg & 0x00ff
        high = (arg >> 8) & 0x00ff
        self.sendCommand(command, low, high)

    ##############################################################################
    def sendCommand(self, command, arg1=None, arg2=None):

        message = command
        if arg1 != None and arg2 == None:
            message = "%s%c" % (message, chr(int(arg1)))
            rospy.loginfo("Sending to controller: [%s %s]" % (format(ord(command), '02x'),
                                                             format(int(arg1), '02x')))

        elif arg1 != None and arg2 != None:
            message = "%s%c%c" % (message, chr(int(arg1)), chr(int(arg2)))
            rospy.loginfo("Sending to controller: [%s %s %s]" % (format(ord(command), '02x'),
                                                             format(int(arg1), '02x'),
                                                             format(int(arg2), '02x')))

        else:
            rospy.loginfo("Sending to controller: [%s]" % format(ord(command), '02x'))

        self.writeUsb(message)

    ##############################################################################
    def sendCommandUI(self, entity, mode):

        if entity in self.ENTITIES:
            entityCode = self.ENTITIES[entity];
        else:
            return False
        
        if mode in self.MODES:
            modeCode = self.MODES[mode]
        else:
            return False
        
        self.sendCommand('7', entityCode, modeCode)
        return True

    ##############################################################################
    def sendCommandDistance(self, distance):

        self.sendCommandWithInteger('d', distance)
        return True

    ##############################################################################
    def sendCommandRotate(self, angle):
        
        self.sendCommandWithInteger('0', angle)
        return True

    ##############################################################################
    def sendCommandStop(self):

        self.sendCommand('s')
        return True

    ##############################################################################
    def sendCmdVersion(self):

        self.sendCommand('y')
        return True

    ##############################################################################
    def writeUsb(self, message):

        self._write_lock.acquire()
        try:
            self.usbCmd.write("%s\r\n" % message)
        except serial.SerialException, e:
            rospy.logerr('USB disconnected. Attempting to re-connect')
            if not self.connectUSB():
                exit(1)

        finally:
            self._write_lock.release()

    ##############################################################################
    def setupReadThread(self):

        self.alive = True
        self._write_lock = threading.Lock()

        self.readThread = threading.Thread(target=self.usbReader)
        self.readThread.setDaemon(True)
        self.readThread.setName('serial->ros')
        self.readThread.start()

    ##############################################################################
    # Run the thread for the serial connection reader
    def usbReader(self):

        while self.alive:
            endMessage = False
            message = ''

            try:
                while not endMessage:
                    c = self.usbCmd.read(1)
                    if c != '\n':
                        message = message + c
                    else:
                        endMessage = True

                if message:
                    message = message.rstrip()
                    if message:
                        if message[0] == '<':
                            rospy.loginfo("Publishing debug message: \"%s\"" % message)
                            self.pubLLDebug.publish(message)
                        else:
                            self.publishLLEvent(message)
                    else:
                        rospy.loginfo('Received empty message')

            except serial.SerialException, e:
                rospy.logerr('USB disconnected. Attempting to re-connect')
                if not self.connectUSB():
                    exit(1)

        self.alive = False

##################################################################################
if __name__ == '__main__':
    rospy.init_node('node_base_controller')
    bc = BaseController()
    bc.run()
