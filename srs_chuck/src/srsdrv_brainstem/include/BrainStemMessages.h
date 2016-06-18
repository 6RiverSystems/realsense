/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <stdint.h>
#include <limits>

#ifndef _MESSAGES_H_
#define _MESSAGES_H_

namespace srs {

enum class BRAIN_STEM_MSG
{
	MESSAGE				= 0x3C, // '<'
	BUTTON_PRESSED			= 0x42, // 'B',
	HARDWARE_INFO			= 0x59, // 'Y',
	ODOMETRY_VELOCITY		= 0x4f, // 'O',
	OPERATIONAL_STATE		= 0x47, // 'G',
	SYSTEM_VOLTAGE			= 0x56, // 'V',
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
	SET_SUSPEND_STATE		= 0x34, // '4'
	SET_TOTE_LIGHTS			= 0x39, // '9'
	SET_VELOCITY			= 0x76, // 'v'
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
	UNKNOWN
};

enum class LED_MODE
{
	OFF 		= 0,
	ON 			= 1,
	BLINK 		= 2,
	GRAB 		= 10,
	PUT 		= 11,
	BRAKE 		= 100,
	TURN 		= 101,
	SELECT 		= 102,
	UNKNOWN
};


enum class MOTION_STATUS
{
	FRONT_E_STOP 		= 0b00000001,
	BACK_E_STOP 		= 0b00000010,
	WIRELESS_E_STOP 	= 0b00000100,
	BUMP_SENSOR			= 0b00001000,
	PAUSE 			= 0b00010000,
	HARD_STOP 			= 0b00100000
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

struct HARDWARE_INFORMATION_DATA
{
	uint8_t 	cmd;
	uint16_t	uniqueId;					// Hardware unique identifier (unique across all robots)
	uint8_t		bodyType; 					// Body type
	uint32_t	configuration; 				// configuration layout
	uint32_t	lifetimeHours; 				// lifetime hours
	uint32_t	lifetimeMeters; 			// lifetime meters
	uint32_t	batteryHours;				// battery hours
	uint32_t	wheelMeters; 				// wheel meters
	char*		pszBrainstemVersion;		// brainstem version
};

struct MOTION_STATUS_DATA
{
	uint8_t		frontEStop:1; 				// front eStop state
	uint8_t		backEStop:1; 				// back eStop state
	uint8_t		wirelessEStop:1; 			// wireless eStop state
	uint8_t		bumpSensor:1; 				// bump sensor state
	uint8_t		pause:1; 					// paused state
	uint8_t		hardStop:1; 				// hard stop state
	uint8_t		reservedMotion:2; 			// reserved
};

struct FAILURE_STATUS_DATA
{
	uint8_t		safetyProcessorFailure:1; 	// safety processor failure
	uint8_t		brainstemFailure:1; 		// brainstem failure
	uint8_t		brainTimeoutFailure:1; 		// brainstem timeout failure
	uint8_t		rightMotorFailure:1; 		// right motor failure
	uint8_t		leftMotorFailure:1; 		// left motor failure
	uint8_t		reservedFailure:3; 			// reserved
};

struct OPERATIONAL_STATE_DATA
{
	uint8_t 			cmd;
	uint32_t			upTime;				// Hardware up time in seconds
	MOTION_STATUS_DATA	motionStatus;		// Motion status
	FAILURE_STATUS_DATA failureStatus;		// Failure status
	uint8_t				suspendState; 		// 0 = unsuspended, suspended otherwise
};

struct SET_OPERATIONAL_STATE_DATA
{
	uint8_t 			cmd;
	MOTION_STATUS_DATA	motionStatus;		// Motion status
};

struct VOLTAGE_DATA
{
	uint32_t	cmd;
	float		voltage;					// Voltage from 0V => 24V
};

struct SUSPEND_DATA
{
	uint8_t cmd;
	uint8_t isSuspended;				// 0 or '0' = unsuspended, suspended otherwise
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

// Back to normal packing
#pragma pack(pop)

}

#endif /* _MESSAGES_H_ */
