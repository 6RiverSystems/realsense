/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <stdint.h>
#include <limits>
#include <uuid/uuid.h>

#ifndef _MESSAGES_H_
#define _MESSAGES_H_

namespace srs {

enum class BRAIN_STEM_MSG
{
    MESSAGE = 0x3C, // '<'
    BUTTON_PRESSED = 0x42, // 'B'
    HARDWARE_INFO = 0x59, // 'Y'
    OPERATIONAL_STATE = 0x47, // 'G'
    SENSOR_FRAME = 0x4F, // 'O'
	RAW_ODOMETRY = 0x52, // 'R'
    SYSTEM_VOLTAGE = 0x56, // 'V'
    UNKNOWN
};

enum class BRAIN_STEM_CMD
{
	CLEAR_MOTION_STATUS		= 0x31, // '1'
	GET_HARDWARE_INFO		= 0x79, // 'y'
	GET_OPERATIONAL_STATE	= 0x67, // 'g'
	HARD_STOP				= 0x68, // 'h'
	PING					= 0x63, // 'c'
	RESET_BATTERY_HOURS		= 0x62, // 'b'
	RESET_WHEEL_METERS		= 0x77, // 'w'
	UPDATE_LIGHT			= 0x37, // '7'
	SET_CONFIGURATION		= 0x66, // 'f'
	SET_MOTION_STATUS		= 0x32, // '2'
	SET_TOTE_LIGHTS			= 0x39, // '9'
	SET_VELOCITY			= 0x76, // 'v'
	SET_VELOCITY_RPM        = 0x72, // 'r'
	STARTUP					= 0x38, // '8'
	SHUTDOWN				= 0x78, // 'x'
	UNKNOWN
};

enum class LED_ENTITIES
{
	TOTE0 		= 0,
	TOTE1 		= 1,
	TOTE2 		= 2,
	TOTE3 		= 3,
	TOTE4 		= 4,
	TOTE5 		= 5,
	TOTE6 		= 6,
	TOTE7 		= 7,
	ACTION 		= 100,
	PAUSE  		= 101,
	TAIL_LEFT  	= 201,
	TAIL_RIGHT 	= 202,
	SCANNER 	= 203,
	UNKNOWN
};

enum class LED_MODE
{
	OFF 		= 0,
	ON 			= 1,
	ESTOP 		= 2,
	GRAB 		= 10,
	PUT 		= 11,
	BRAKE 		= 100,
	TURN 		= 101,
	SELECT 		= 102,
	RAPID_BLINK = 103,
	UNKNOWN
};


enum MOTION_STATUS
{
	FRONT_E_STOP 		= 0,
	BACK_E_STOP 		= 1,
	WIRELESS_E_STOP 	= 2,
	BUMP_SENSOR			= 3,
	PAUSE 				= 4,
	HARD_STOP 			= 5
};

enum FAILURE_STATUS
{
	SAFETY_PROCESSOR	= 0,
	BRAINSTEM			= 1,
	BRAINSTEM_TIMEOUT	= 2,
	RIGHT_MOTOR			= 3,
	LEFT_MOTOR			= 4
};

// We need to make sure that these are byte packed
#pragma pack(push, 1)

struct COMMAND_DATA
{
	uint8_t cmd;
};

struct LIGHT_UPDATE_DATA
{
	uint8_t 	cmd;
	uint8_t 	entitiy; 				// Light identifier - see LED_ENTITIES
	uint8_t 	mode;   				// Light mode - see LED_MODE
};

struct MOTION_STATUS_DATA
{
	bool		frontEStop; 			// front eStop state
	bool		backEStop; 				// back eStop state
	bool		wirelessEStop; 			// wireless eStop state
	bool		bumpSensor; 			// bump sensor state
	bool		pause; 					// paused state
	bool		hardStop; 				// hard stop state
};

struct FAILURE_STATUS_DATA
{
	bool		safetyProcessorFailure; // safety processor failure
	bool		brainstemFailure; 		// brainstem failure
	bool		brainTimeoutFailure; 	// brainstem timeout failure
	bool		rightMotorFailure; 		// right motor failure
	bool		leftMotorFailure; 		// left motor failure
};


struct OPERATIONAL_STATE_DATA
{
	uint8_t 	cmd;
	uint32_t	upTime;					// Hardware up time in seconds
	uint8_t		motionStatus;			// Motion status
	uint8_t 	failureStatus;			// Failure status
	uint8_t		suspendState; 			// 0 = unsuspended, suspended otherwise
};

struct SET_OPERATIONAL_STATE_DATA
{
	uint8_t 	cmd;
	uint8_t		motionStatus;			// Motion status
};

struct VOLTAGE_DATA
{
	uint8_t		cmd;
	float		voltage;				// Voltage from 0V => 24V
};

struct SUSPEND_DATA
{
	uint8_t 	cmd;
	uint8_t 	isSuspended;			// 0 or '0' = unsuspended, suspended otherwise
};

struct VELOCITY_DATA
{
	uint8_t		cmd;
	float		linear_velocity;
	float		angular_velocity;
};

struct ODOMETRY_DATA
{
	uint8_t		cmd;
	uint32_t	timestamp;
	float		linear_velocity;
	float		angular_velocity;
};

struct ODOMETRY_RPM_DATA
{
	uint8_t		cmd;
	float		left_wheel_rpm;
	float		right_wheel_rpm;
};

// Back to normal packing
#pragma pack(pop)

}

#endif /* _MESSAGES_H_ */
