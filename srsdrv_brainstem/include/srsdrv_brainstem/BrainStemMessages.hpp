/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <stdint.h>
#include <limits>
#include <uuid/uuid.h>

// Define the macros for the beginning and end of a
// brainstem message. The macro automatically define the
// strict packing pragma and the struct for the
// message data
#define HW_MESSAGE_BEGIN(name) \
	_Pragma("pack(push, 1)") \
	struct name \
	{

	#define HW_MESSAGE_END \
	}; \
	_Pragma("pack(pop)")

namespace srs {

// Brainstem => Brain
enum class BRAIN_STEM_MSG : uint8_t
{
	MESSAGE = 0x3C, // '<'
	BUTTON_PRESSED = 0x42, // 'B'
	HARDWARE_INFO = 0x59, // 'Y'
	OPERATIONAL_STATE = 0x47, // 'G'
	SENSOR_FRAME = 0x4F, // 'O'
	POWER_STATE = 0x41, // 'A'
	RAW_ODOMETRY = 0x52, // 'R'
	ODOMETRY_POSE = 0x50, // 'P'
	SYSTEM_VOLTAGE = 0x56, // 'V'
	LOG = 0x4C, // 'L'
	UNKNOWN
};

// Brain => Brainstem
enum class BRAIN_STEM_CMD : uint8_t
{
	CLEAR_MOTION_STATUS     = 0x31, // '1'

	GET_HARDWARE_INFO       = 0x79, // 'y'
	GET_OPERATIONAL_STATE   = 0x67, // 'g'

	HARD_STOP               = 0x68, // 'h'

	PING                    = 0x63, // 'c'

	RESET_BATTERY_HOURS     = 0x62, // 'b'
	RESET_WHEEL_METERS      = 0x77, // 'w'

    SET_ALLOWED_MAX_VELOCITY= 0x6D, // 'm'
    SET_CONFIGURATION       = 0x66, // 'f'
    SET_DIMENSION           = 0x64, // 'd'
	SET_MOTION_STATUS       = 0x32, // '2'
	SET_TOTE_LIGHTS         = 0x39, // '9'
	SET_BODY_LIGHTS         = 0x6C, // 'l'
	SET_VELOCITY            = 0x76, // 'v'
	SET_VELOCITY_RPM        = 0x72, // 'r'
	SHUTDOWN                = 0x78, // 'x'
	SOUND_BUZZER            = 0x73, // 's'
	STARTUP                 = 0x38, // '8'

	FORCE_WATCHDOG_TIMEOUT	= 0x77, // 'w'

	UPDATE_LIGHT            = 0x37, // '7'
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
	SOFT_POWER	= 102,
	TAIL_LEFT  	= 201,
	TAIL_RIGHT 	= 202,
	SCANNER 	= 203,
	UNKNOWN
};

enum class LED_MODE : uint8_t
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

enum class BODY_LIGHTS_ENTITIES
{
	ACTION 		= 100,
	PAUSE  		= 101,
	SOFT_POWER	= 102,
	TAIL_LEFT  	= 201,
	TAIL_RIGHT 	= 202,
	SCANNER 	= 203,
	HEAD_LEFT	= 204,
	HEAD_RIGHT	= 205,
	UNKNOWN
};

enum class LED_COMMAND : uint8_t
{
	OFF 		= 0,
	SOLID 		= 1,
	RAMP 		= 2,
	RAMP_BIDIR 	= 3,
	BLINK 		= 4,
	SAWTOOTH_RIGHT 	= 5,
	SAWTOOTH_LEFT 	= 6,
	GRADIENT 	= 7,
	UNKNOWN
};

enum MOTION_STATUS : uint8_t
{
	FRONT_E_STOP = 0,
	BACK_E_STOP = 1,
	WIRELESS_E_STOP = 2,
	BUMP_SENSOR = 3,
	FREE_SPIN = 4,
	HARD_STOP = 5
};

enum FAILURE_STATUS : uint8_t
{
    SAFETY_PROCESSOR_FAILURE = 0,
    BRAINSTEM_FAILURE = 1,
    BRAIN_TIMEOUT = 2,
    RIGHT_MOTOR_CONTROLLER = 3,
    LEFT_MOTOR_CONTROLLER = 4,
    RPM_MESSAGE_TIMEOUT = 5,
    VELOCITY_VIOLATION = 6
};

// A struct which includes entity attributes
typedef struct
{
	LED_COMMAND lightCmd;
	uint8_t startColor[3];
	uint8_t endColor[3];
	float frequency;
} ENTITY_STATE;

}
