/*
 * BrainStemMessages.h
 *
 *  Created on: Apr 7, 2016
 *      Author: dan
 */

#include <stdint.h>
#include <limits>

#ifndef _MESSAGES_H_
#define _MESSAGES_H_

namespace srs {

enum class BRAIN_STEM_MSG
{
	MESSAGE		= '<',
	STOP		= 'S',
	BUTTON		= 'B',
	ODOMETRY	= 'O',
	PID			= 'P',
	UNKNOWN
};

enum class BRAIN_STEM_CMD
{
	FORWARD = 'f',
	BACKWARD = 'b',
	RIGHT = 'r',
	LEFT = 'l',
	FORWARD_TIMED = 't',
	BACKWARD_TIMED = 'u',
	LEFT_TIMED = 'z',
	RIGHT_TIMED = 'e',
	CAM = 'v',
	CAM_RELEASE = 'w',
	CAM_HORIZ_SET = 'm',
	STOP = 's',
	HARD_STOP = 'h',
	GET_VERSION = 'y',
	GET_FIRMWARE = 'x',
	FWD_FLOOD_LIGHT = 'q',
	REAR_FLOOD_LIGHT = 'o',
	SET_PID = 'p',
	ODOMETRY_START = 'i',
	ODOMETRY_STOP = 'j',
	ODOMETRY_REPORT = 'k',
	PING = 'c',
	DISTANCE = 'd',
	LIGHT_UPDATE = '7',
	ROTATE = '0',
	STARTUP = '8',
	SLINGSHOT_RIGHT = '6',
	SLINGSHOT_LEFT = '5',
	SUSPEND_UPDATE_STATE = '4',
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

struct ODOMETRY_DATA
{
	uint8_t		cmd;
	uint32_t	left_encoder;
	uint32_t	right_encoder;
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
