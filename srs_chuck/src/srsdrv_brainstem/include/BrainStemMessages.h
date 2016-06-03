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
	STOP				= 0x53, // 'S'
	BUTTON				= 0x42, // 'B',
	HARDWARE_INFO		= 0x59, // 'Y',
	ODOMETRY_VELOCITY	= 0x4f, // 'O',
	OPERATIONAL_STATE	= 0x47, // 'G',
	SYSTEM_VOLTAGE		= 0x56, // 'V',
	UNKNOWN
};

enum class BRAIN_STEM_CMD
{
	CLEAR_MOTION_STATUS	= 0x31, // '1'
	GET_VERSION			= 0x79, // 'y'
	GET_STATE			= 0x67, // 'g'
	HARD_STOP			= 0x68, // 'h'
	PING				= 0x63, // 'c'
	RESET_BATTERY_HOURS	= 0x62, // 'b'
	RESET_WHEEL_METERS	= 0x77, // 'w'
	LIGHT_UPDATE		= 0x37, // '7'
	SET_CONFIGURATION	= 0x66, // 'f'
	SET_MOTION_STATUS	= 0x32, // '2'
	SET_SUSPEND_STATE	= 0x34, // '4'
	SET_TOTE_LIGHTS		= 0x39, // '9'
	SET_VELOCITY		= 0x76, // 'v'
	STARTUP				= 0x38, // '8'
	SHUTDOWN			= 0x78, // 'x'
	UNKNOWN
};

enum class ENTITIES
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

// We need to make sure that these are byte packed
#pragma pack(push, 1)

struct MOVE_DATA
{
	uint8_t cmd;
	uint8_t left_speed;
	uint8_t right_speed;
};

struct TIMED_MOVE_DATA
{
	uint8_t cmd;
	uint8_t left_speed;
	uint8_t right_speed;
	uint16_t move_time_ms;
};

struct MOVE_DISTANCE_DATA
{
	uint8_t cmd;
	int16_t dist_mm;
};

struct ROTATE_DATA
{
	uint8_t cmd;
	int16_t deg_tenths; // 10ths of degrees.  Positive is right, negative is left
};

struct SLINGSHOT_DATA
{
	uint8_t	cmd;
	uint16_t dist_mm; // after turn is completed
};

struct LIGHT_UPDATE_DATA
{
	uint8_t cmd;
	uint8_t entitiy; // Light identifier - see led_entities
	uint8_t mode;   // Light mode - see led_mode
};

struct SUSPEND_DATA
{
	uint8_t cmd;
	uint8_t isSuspended; // 0 or '0' = unsuspended, suspended otherwise
};

struct PID_PARAMETERS_DATA
{
	uint8_t cmd;
	float p;
	float i;
	float d;
	float proj_time;
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

struct PID_DATA
{
	float x_est;
	float y_est;
	float v_est;
	float theta_est;
	float omega_est;
	float v_des;
	float omega_des;
	float v_left_des;
	float v_right_des;
};

// Back to normal packing
#pragma pack(pop)

}

#endif /* _MESSAGES_H_ */
