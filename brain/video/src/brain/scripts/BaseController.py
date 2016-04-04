#!/usr/bin/env python
import rospy
import math
import sys, os, time
import threading
import serial
import time
import struct

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

class CommandPacketizer:
    def __init__(self, command, payload = bytearray('')):
        self.bytesArray = bytearray('')
        self.command = command
        self.payload = payload
        
    def generateBytes(self, generateCRC = None, includeLength = True, teminatingStr = None, escapeChar = None, charsToEscape = []):
        #add the tuple for the header        
        self.bytesArray.append(struct.pack('c', self.command))

        #add the length
        if(includeLength == True):
            self.bytesArray.append(struct.pack('c', sys.getsizeof(self.payload)))
        
        self.bytesArray += self.payload

        # generate the crc
        crc = 0
        for onebyte in self.bytesArray:
            crc += onebyte

        #Add the CRC
        if (generateCRC):
            crcChar = 0
            
            # sum all of the bytes and put sum at 
            if (generateCRC is "POSITIVE"):
                crcChar = chr(crc&0xFF)
            
            elif (generateCRC is "NEGATIVE"):
                crcChar = chr((-crc)&0xFF) 

            self.bytesArray.append(crcChar) 

        # NOTE: Escape characters are NOT in the CRC calculation
        for charToEscape in charsToEscape:
            self.bytesArray = self.bytesArray.replace(charToEscape, escapeChar + charToEscape)

        if teminatingStr is not None:
            for c in teminatingStr:
                self.bytesArray.append(ord(c))
        
        return self.bytesArray
        
    def __str__(self):
        mystr = ''
        if  hasattr(self, 'bytesArray'):
            first = True
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
#     CMD_SPOT_LIGHT = 'p'
    CMD_SET_PID = 'p'
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
        self.pubLLSensors = rospy.Publisher('/ll_sensors', String, queue_size=1000)

        self.pubXEst = rospy.Publisher('/pid/x_est', Float32, queue_size=1000)
        self.pubYEst = rospy.Publisher('/pid/y_est', Float32, queue_size=1000)
        self.pubVEst = rospy.Publisher('/pid/v_est', Float32, queue_size=1000)
        self.pubThetaEst = rospy.Publisher('/pid/theta_est', Float32, queue_size=1000)
        self.pubOmegaEst = rospy.Publisher('/pid/omega_est', Float32, queue_size=1000)
        self.pubVDes = rospy.Publisher('/pid/v_des', Float32, queue_size=1000)
        self.pubOmegaDes = rospy.Publisher('/pid/omega_des', Float32, queue_size=1000)
        self.pubVLeftDes = rospy.Publisher('/pid/v_left_des', Float32, queue_size=1000)
        self.pubVRightDes = rospy.Publisher('/pid/v_right_des', Float32, queue_size=1000)

        # generate a reverse lookup table
        self.REV_ENTITIES = {v:k for k, v in self.ENTITIES.iteritems()}        
        
        self.usbCmd = serial.Serial()
        self.usbCmd.port = '/dev/malg'
        self.usbCmd.baudrate = 115200
        self.usbCmd.parity = 'N'
        self.usbCmd.rtscts = False
        self.usbCmd.xonxoff = False
        self.usbCmd.timeout = 0

        self.controllerFault = False

        self.messageBuffer = []

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
            # look for a function called "processCommandFOO" where FOO is the command that was specified
            # then pass that function the rest of the command arguements
            if command[0] != '': 
                try:
                    methodToCall = getattr(self, 'processCommand' + command[0])
                    methodToCall(command[1:])

                except (AttributeError) as e:
                    rospy.logerr('No function named "processCommand' + command[0] + '" ' + repr(e))
 #                   print ('No function named "processCommand' + command[0] + '"')
                except (TooManyBytes, TooFewBytes, IllegalValue) as e:
                    rospy.logerr('%s: %s for command %s' % (e.__class__.__name__, e.value, command))
 #                    print '%s: %s for command %s' % (e.__class__.__name__, e.value, command)
        return 
        
    ##############################################################################
    # Process the packets coming from the MFP
    def processMFPCommand(self, message):
        try:
            event = ''
            publisher = None

#            rospy.loginfo("Received message: [%s]" % message)
            
            # Look for a dangerous error - shut things down - is this a good idea?
            if message[0:10] == '<MSG Error':
                self.controllerFault = True
                rospy.logerr('Controller disabled')
 
            # Debug Message
            if message[0] == '<':
                publisher = self.pubLLDebug
                event = message[1:-1]
                
            # STOP event
            elif message[0] == 'S':
                publisher = self.pubLLEvent
                event = 'ARRIVED'
     
            # BUTTON event
            elif message[0] == 'B':
                publisher = self.pubLLEvent
                entityName = self.REV_ENTITIES[ord(message[1])]
                event = "UI %s" % entityName
            
            # ODOMETRY event
            elif message[0] == 'O':
                Rord = ord(message[1]) + ord(message[2]) * 256 + ord(message[3]) * 256 * 256 + ord(message[4]) * 256 * 256 * 256
                Lord = ord(message[5]) + ord(message[6]) * 256 + ord(message[7]) * 256 * 256 + ord(message[8]) * 256 * 256 * 256
                
                event = "O %i,%i @ %s" % (Rord, Lord, time.time())
                publisher = self.pubLLSensors
           
            # PID event
            elif message[0] == 'P':
                pidData = struct.unpack('cfffffffff', message);

                pubXEst.publish("X_EST %f" % pidData[1])
                pubYEst.publish("Y_EST %f" % pidData[2])
                pubVEst.publish("V_EST %f" % pidData[3])
                pubThetaEst.publish("THETA_EST %f" % pidData[4])
                pubOmegaEst.publish("OMEGA_EST %f" % pidData[5])
                pubVDes.publish("V_DES %f" % pidData[6])
                pubOmegaDes.publish("OMEGA_DES %f" % pidData[7])
                pubVLeftDes.publish("V_LEFT_DES %f" % pidData[8])
                pubVRightDes.publish("V_RIGTH_DES %f" % pidData[9])

            # unknown event
            else:
                rospy.logwarn('Unknown MFP command: %s' % message)

            if publisher:
                # dont spam the log with sensor data
                if publisher != self.pubLLSensors:
                    rospy.loginfo("[%s]: [%s]" % (publisher.name, event))
                publisher.publish(event)
        except IndexError:
            if(len(message) == 0):
                rospy.logwarn("Call to processMFPCommand with zero length message [%s] - ignoring" % message)
            else:
                rospy.logwarn("Call to processMFPCommand of command %s with length of only %i - ignoring" % (repr(message), len(message)))

    ##############################################################################
    def sendCommand(self, commandPacketizer):
        message = commandPacketizer.generateBytes(generateCRC = "NEGATIVE", includeLength = False, teminatingStr = '\n', escapeChar = '\\', charsToEscape = ['\\','\n'])
        rospy.loginfo("Sending to controller: [%s]" % commandPacketizer)

        self.writeSerialPort(message)

    ##############################################################################
    def processCommandUI(self, commandPayload):
        if len(commandPayload) != 2:
            raise IllegalValue('Invalid number of arguments for UI command %s' % repr(commandPayload))
        
        entity = commandPayload[0]
        mode = commandPayload[1]

        if entity not in self.ENTITIES:
            raise IllegalValue('The entity %i is not valid' % entity)
            
        if mode not in self.MODES:
            raise IllegalValue('The mode %i is not valid' % mode)

        payload = struct.pack('BB', self.ENTITIES[entity], self.MODES[mode])
        cp = CommandPacketizer(self.CMD_LIGHT_UPDATE, payload)
        self.sendCommand(cp)

    ##############################################################################
    def processCommandSTARTUP(self, commandPayload):
        cp = CommandPacketizer(self.CMD_STARTUP)
        self.sendCommand(cp)

    ##############################################################################
    def processCommandDISTANCE(self, commandPayload):
        if len(commandPayload) != 1:
            raise IllegalValue('Invalid number of arguments for DISTANCE command %s' % repr(commandPayload))

        distance = int(commandPayload[0])
        payload = struct.pack('H', distance)
        cp = CommandPacketizer(self.CMD_DISTANCE, payload)
        self.sendCommand(cp)

    ##############################################################################
    def processCommandROTATE(self, commandPayload):
        if len(commandPayload) != 1:
            raise IllegalValue('Invalid number of arguments for ROTATE command %s' % repr(commandPayload))

        angle = int(commandPayload[0])
        payload = struct.pack('H', angle)
        cp = CommandPacketizer(self.CMD_ROTATE, payload)
        self.sendCommand(cp)

    ##############################################################################
    def processCommandSTOP(self, commandPayload):
        cp = CommandPacketizer(self.CMD_STOP)
        self.sendCommand(cp)

    ##############################################################################
    def processCommandTURN(self, commandPayload):
        if len(commandPayload) != 2:
            raise IllegalValue('Invalid number of arguments for TURN command %s' % repr(commandPayload))

        direction = commandPayload[0]
        distance = int(commandPayload[1])

        command = ''
        if direction == 'R':
            command = self.CMD_SLINGSHOT_RIGHT
        elif direction == 'L':
            command = self.CMD_SLINGSHOT_LEFT
        else:
            raise IllegalValue('Invalid turn direction %c', direction)

        payload = struct.pack('H', distance)
        cp = CommandPacketizer(command, payload)
        self.sendCommand(cp)

    ##############################################################################
    def processCommandVERSION(self, commandPayload):
        cp = CommandPacketizer(self.CMD_GET_VERSION)
        self.sendCommand(cp)

    ##############################################################################
    def processCommandPAUSE(self, commandPayload):
        if len(commandPayload) != 1:
            raise IllegalValue('Invalid number of arguments for PAUSE command %s' % repr(commandPayload))

        pauseValue = 0
        if commandPayload[0] == 'ON':
            pauseValue = 1
        elif commandPayload[0] == 'OFF':
            pauseValue = 0
        else:
            raise IllegalValue('Pause requires either ON or OFF but was given %s' % commandPayload[0])
            
        payload = struct.pack('B', pauseValue)
        cp = CommandPacketizer(self.CMD_SUSPEND_UPDATE_STATE, payload)
        self.sendCommand(cp)
        
    ##############################################################################
    def processCommandREENABLE(self, commandPayload):
        self.controllerFault = False
        
    ##############################################################################
    def processCommandSET_PID(self, commandPayload):
        if len(commandPayload) != 4:
            raise IllegalValue('Invalid pid data %s' % repr(commandPayload))

        Kp = float(commandPayload[0])
        Ki = float(commandPayload[1])
        Kd = float(commandPayload[2])
        proj_time = float(commandPayload[3])
        payload = struct.pack('ffff', Kp, Ki, Kd, proj_time)
        rospy.logerr('Setting PID from command: Kp: ' + str(Kp) + ', Ki: ' + str(Ki) + ', Kd: ' + str(Kd) + ', time: ' + str(proj_time))
        cp = CommandPacketizer(self.CMD_SET_PID, payload)
        self.sendCommand(cp)


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
            
            except rospy.exceptions.ROSInterruptException:
                pass

        return True

    ##############################################################################
    def writeSerialPort(self, message):
        if not self.controllerFault:
            try:
                self.usbCmd.write("%s" % message)
            except serial.SerialException, e:
                rospy.logerr('USB disconnected. Attempting to re-connect')
                if not self.connectUSB():
                    exit(1)
        else:
            rospy.logerr('Controller disabled')

    ##############################################################################
    # Parse the current message buffer for commands
    def parseMessage(self):
        result = []
        message = ''
        escapedState = False
        for c in self.messageBuffer:
            if c == '\\':
                if escapedState:
                    message += c
                    escapedState = False
                else:
                    escapedState = True
            elif escapedState or c != '\n':
                message += c
                escapedState = False
            else:
                self.processMFPCommand(message)
                message = ''

        # remainder of message
        self.messageBuffer = message

    ##############################################################################
    # see if any data is available on the serial connection
    def readSerialPort(self):
        try:
            self.messageBuffer += self.usbCmd.read(4096)
            self.parseMessage();
 
        except serial.SerialException, e:
            rospy.logerr('USB disconnected. Attempting to re-connect')
            if not self.connectUSB():
                exit(1)

    ##############################################################################
    # Run the node
    def run(self):

        rospy.loginfo('Base Controller Node - video version')

        if not self.connectUSB():
            exit(1)

        try:
            while not rospy.is_shutdown():
                self.readSerialPort()
                self.rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass
        
##################################################################################
if __name__ == '__main__':
    rospy.init_node('node_base_controller')
    bc = BaseController()
    bc.run()
