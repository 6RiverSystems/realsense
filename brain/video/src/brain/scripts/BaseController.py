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
        'TOTE0':        0,
        'TOTE1':        1,
        'TOTE2':        2,
        'TOTE3':        3,
        'TOTE4':        4,
        'TOTE5':        5,
        'TOTE6':        6,
        'TOTE7':        7,

        'ACTION':     100,
        'PAUSE':      101,

        'TAIL_LEFT':  201,
        'TAIL_RIGHT': 202
    }
    
    MODES = {
        'OFF':          0,
        'ON':           1,
        
        'TAKE':        10,
        'PICK':        10,
        'GRAB':        10,
        'PUT':         11,
        
        'BRAKE':      100,
        'TURN':       101,
        'SELECT':     102,
    }
    
    ##############################################################################
    # Initialize the node properties
    def __init__(self):

        self.rate = rospy.Rate(100)
        self.subCmdLL = rospy.Subscriber('/cmd_ll', String, self.cbCmdLL, queue_size=50)
        self.pubLLEvent = rospy.Publisher('/ll_event', String, queue_size=1)
        self.pubLLDebug = rospy.Publisher('/ll_debug', String, queue_size=1)

        # generate a reverse lookup table
        self.REV_ENTITIES = {v:k for k, v in self.ENTITIES.iteritems()}        
        
        self.usbCmd = serial.Serial()
        self.usbCmd.port = '/dev/malg'
        self.usbCmd.baudrate = 115200
        self.usbCmd.parity = 'N'
        self.usbCmd.rtscts = False
        self.usbCmd.xonxoff = False
        self.usbCmd.timeout = 1

        self.latestCommand = ''
        self.controllerFault = False

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

                # renable
                if command[0] == 'REENABLE':
                    self.controllerFault = False
                    success = True

                # stop
                if command[0] == 'STOP':
                    success = self.sendCommandStop()
                
                # distance <distance [mm]>
                elif command[0] == 'DISTANCE':
                    distance = int(command[1])
                    if distance == 0:
                        success = self.sendCommandStop()
                    else:
                        success = self.sendCommandDistance(distance)
                
                # rotate <angle [1/10deg]>
                elif command[0] == 'ROTATE':
                    angle = int(command[1])
                    success = self.sendCommandRotate(angle)
                
                # ui <entity> <mode>
                elif command[0] == 'UI':
                    entity = command[1]
                    mode = command[2]
                    success = self.sendCommandUI(entity, mode)
                
                # turn <L|R> <distance [mm]>
                elif command[0] == 'TURN':
                    direction = command[1]
                    distance = int(command[2])
                    success = self.sendCommandTurn(direction, distance)
                
                # startup: startup sequence
                elif command[0] == 'STARTUP':
                    success = self.sendCommandStartup()

                # version
                elif command[0] == 'VERSION':
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
        if message[0] == 'S':
            event = 'ARRIVED'
 
        elif message[0] == 'B':
            entityName = self.REV_ENTITIES[ord(message[1])];
            event = "UI %s" % entityName

        rospy.loginfo("Publishing LL event: [%s]" % event)
        self.pubLLEvent.publish(event)

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

        marg1 = chr(0)
        marg2 = chr(0)
        if arg1 != None:
            marg1 = chr(arg1)
        if arg2 != None:
            marg2 = chr(arg2)

        crc = chr((ord(command) + ord(marg1) + ord(marg2)) % 256)

        message = "%c%c%c%c" % (command, marg1, marg2, crc)
        rospy.loginfo("Sending to controller: [%s %s %s %s]" % (
                                                             format(ord(command), '02x'),
                                                             format(ord(marg1), '02x'),
                                                             format(ord(marg2), '02x'),
                                                             format(ord(crc), '02x')))
        self.latestCommand = message
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
    def sendCommandStartup(self):

        self.sendCommand('8')
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
    def sendCommandTurn(self, direction, distance):

        if direction == 'R':
            self.sendCommandWithInteger('6', distance)
        elif direction == 'L':
            self.sendCommandWithInteger('4', distance)
        else:
            return False

        return True

    ##############################################################################
    def sendCmdVersion(self):

        self.sendCommand('y')
        return True

    ##############################################################################
    def writeUsb(self, message):

        if not self.controllerFault:
            self._write_lock.acquire()
            try:
                self.usbCmd.write("%s\r\n" % message)
            except serial.SerialException, e:
                rospy.logerr('USB disconnected. Attempting to re-connect')
                if not self.connectUSB():
                    exit(1)
    
            finally:
                self._write_lock.release()
        else:
            rospy.logerr('Controller disabled')

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
                        rospy.loginfo("Received message: [%s]" % message)
                        
                        if message[0:10] == '<MSG Error':
                            self.controllerFault = True
                            rospy.logerr('Controller disabled')
                            
                        elif message[0] == '<':
                            rospy.loginfo("Publishing debug message: [%s]" % message)
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
