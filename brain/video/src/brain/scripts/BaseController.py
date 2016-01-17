#!/usr/bin/env python
import rospy
import math
import sys, os, time
import threading
import serial

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist

class TooManyBytes(Exception):
    def __init__(self, value):
         self.value = value
    def __str__(self):
        return repr(self.value)    

class TooFewBytes(Exception):
    def __init__(self, value):
         self.value = value
    def __str__(self):
        return repr(self.value)    
            
class IllegalValue(Exception):
    def __init__(self, value):
         self.value = value
    def __str__(self):
        return repr(self.value)       
    
class IllegalCommand(Exception):
    def __init__(self, value):
         self.value = value
    def __str__(self):
        return repr(self.value)       
        
class CommandPacketizer:
    def __init__(self, command, maxPayloadSize = 2):
        self.command = command
        self.maxPayloadSize = maxPayloadSize
        self.currPayloadSize = 0
        self.payloadElementList = []
    
    def append(self, payloadElement, numBytes = None):
        if (isinstance(payloadElement, tuple) and len(payloadElement) == 2):
            (value, numBytes) = payloadElement
        elif (not isinstance(payloadElement, tuple) and (numBytes != None)):
            value = payloadElement
            payloadElement = (value, numBytes)
        else:
            raise ValueError('append either needs one two-element tuple or two values')
            
        self.currPayloadSize += numBytes
        if(self.currPayloadSize > self.maxPayloadSize):
            raise TooManyBytes('more bytes added than payload can hold')
        self.payloadElementList.append(payloadElement)
        
    def generateBytes(self, generateCRC = True, includeLength = True, padWithZeros = False, teminatingStr = None):
        #if we need to pad bytes, then do it now
        if padWithZeros:
            self.append(0, self.maxPayloadSize - self.currPayloadSize)
        
        #add the tuple for the header
        self.payloadElementList.insert(0,(self.command, 1))
        
        #add the length
        if(includeLength == True):
            self.payloadElementList.insert(1,(self.currPayloadSize, 1))
        
        #generate the bytes for the message
        self.bytesArray = bytearray('')
        crc = 0;
        for (value, bytes) in self.payloadElementList:
            for i in range(0,bytes):
                if(type(value) is str):
                    value = ord(value)
                onebyte = value & 0xFF
                crc += onebyte
                self.bytesArray.append(chr(onebyte))
                value = value >> 8
            if value != 0:
                raise TooFewBytes('can not fit value in bytes provided')

        #Add the CRC
        if generateCRC:
            self.bytesArray.append(chr(crc&0xFF)) 
        
        if teminatingStr is not None:
            for c in teminatingStr:
                self.bytesArray.append(ord(c))
        
        return self.bytesArray
        
    def __str__(self):
        mystr = ''
        if  hasattr(self, 'bytesArray'):
            first = True;
            for byte in self.bytesArray:
                if first:
                    first = False
                    mystr = '%s0x%02X' % (mystr, byte)
                else:    
                    mystr = '%s 0x%02X' % (mystr, byte)
        return mystr
        
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
    
    CMD_FORWARD = 'f'
    CMD_BACKWARD = 'b'
    CMD_RIGHT = 'r'
    CMD_LEFT = 'l'
    CMD_FORWARD_TIMED = 't'
    CMD_BACKWARD_TIMED = 'u'
    CMD_LEFT_TIMED = 'z'
    CMD_RIGHT_TIMED = 'e'
    CMD_CAM = 'v'
    CMD_CAM_RELEASE = 'w'
    CMD_CAM_HORIZ_SET = 'm'
    CMD_STOP = 's'
    CMD_HARD_STOP = 'h'
    CMD_GET_VERSION = 'y'
    CMD_GET_FIRMWARE = 'x'
    CMD_FWD_FLOOD_LIGHT = 'q'
    CMD_REAR_FLOOD_LIGHT = 'o'
    CMD_SPOT_LIGHT = 'p'
    CMD_ODOMETRY_START = 'i'
    CMD_ODOMETRY_STOP = 'j'
    CMD_ODOMETRY_REPORT = 'k'
    CMD_PING = 'c'
    CMD_DISTANCE = 'd'
    CMD_LIGHT_UPDATE = '7'
    CMD_ROTATE = '0'
    CMD_STARTUP = '8'
    CMD_SLINGSHOT_RIGHT = '6'
    CMD_SLINGSHOT_LEFT = '5'
    CMD_SUSPEND_UPDATE_STATE = '4'
    
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
                try:
                    # renable
                    if command[0] == 'REENABLE':
                        self.controllerFault = False

                    # stop
                    if command[0] == 'STOP':
                        self.sendCommandStop()
                    
                    # distance <distance [mm]>
                    elif command[0] == 'DISTANCE':
                        distance = int(command[1])
                        if distance == 0:
                            self.sendCommandStop()
                        else:
                            self.sendCommandDistance(distance)
                    
                    # rotate <angle [1/10deg]>
                    elif command[0] == 'ROTATE':
                        angle = int(command[1])
                        self.sendCommandRotate(angle)
                    
                    # ui <entity> <mode>
                    elif command[0] == 'UI':
                        entity = command[1]
                        mode = command[2]
                        self.sendCommandUI(entity, mode)
                    
                    # turn <L|R> <distance [mm]>
                    elif command[0] == 'TURN':
                        direction = command[1]
                        distance = int(command[2])
                        self.sendCommandTurn(direction, distance)
                    
                    # startup: startup sequence
                    elif command[0] == 'STARTUP':
                        self.sendCommandStartup()

                    # version
                    elif command[0] == 'VERSION':
                        self.sendCmdVersion()
                    # empty command - probably last semi colon
                    elif command[0] == '':    
                        pass # this is just a parsing issue with the trailing semicolon
                    else:
                        raise IllegalCommand('Unknown Command[%s]' % command[0])
       
                except (TooManyBytes, TooFewBytes, IllegalValue, IllegalCommand) as e:
                    rospy.logerr('%s: %s for command %s' % (e.__class__.__name__, e.value, command))

                    
                    
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
    def sendCommand(self, commandPacketizer):
        message = commandPacketizer.generateBytes(True, False, True, '\r\n')
        rospy.loginfo("Sending to controller: [%s]" % commandPacketizer)

        self.latestCommand = message
        self.writeUsb(message)

    ##############################################################################
    def sendCommandUI(self, entity, mode):

        if entity not in self.ENTITIES:
            raise IllegalValue('The entity %i is not valid' % entity);
            
        if mode not in self.MODES:
            raise IllegalValue('The mode %i is not valid' % mode);
               
        cp = CommandPacketizer(self.CMD_LIGHT_UPDATE)
        cp.append(self.ENTITIES[entity],1)
        cp.append(self.MODES[mode],1)
        self.sendCommand(cp)

    ##############################################################################
    def sendCommandStartup(self):
        cp = CommandPacketizer(self.CMD_STARTUP)
        self.sendCommand(cp)

    ##############################################################################
    def sendCommandDistance(self, distance):
        cp = CommandPacketizer(self.CMD_DISTANCE)
        cp.append(distance, 2)
        self.sendCommand(cp)
        
    ##############################################################################
    def sendCommandRotate(self, angle):
        cp = CommandPacketizer(self.CMD_ROTATE)
        cp.append(angle, 2)
        self.sendCommand(cp)

    ##############################################################################
    def sendCommandStop(self):
        cp = CommandPacketizer(self.CMD_STOP)
        self.sendCommand(cp)

    ##############################################################################
    def sendCommandTurn(self, direction, distance):

        if direction == 'R':
            cp = CommandPacketizer(self.CMD_SLINGSHOT_RIGHT)
        elif direction == 'L':
            cp = CommandPacketizer(self.CMD_SLINGSHOT_LEFT)
        else:
            raise IllegalValue('Invalid turn direction %c', direction)

        cp.append(distance, 2)
        self.sendCommand(cp)

    ##############################################################################
    def sendCmdVersion(self):
        cp = CommandPacketizer(self.CMD_GET_VERSION)
        self.sendCommand(cp)

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
