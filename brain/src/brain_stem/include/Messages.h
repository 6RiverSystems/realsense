/*
 * BrainStemMessages.h
 *
 *  Created on: Apr 7, 2016
 *      Author: dan
 */

#include <stdint.h>

#ifndef _MESSAGES_H_
#define _MESSAGES_H_

namespace srs {

enum class BRAIN_STEM_MESSAGE
{
	MESSAGE		= '<',
	STOP		= 'S',
	BUTTON		= 'B',
	ODOMETRY	= 'O',
	PID			= 'P'
};

enum class ENTITIES
{
	LIGHT_TOTE0 = 0,
	LIGHT_TOTE1 = 1,
	LIGHT_TOTE2 = 2,
	LIGHT_TOTE3 = 3,
	LIGHT_TOTE4 = 4,
	LIGHT_TOTE5 = 5,
	LIGHT_TOTE6 = 6,
	LIGHT_TOTE7 = 7,
	LIGHT_ACTION = 100,
	LIGHT_PAUSE  = 101,
	LIGHT_TAIL_LEFT  = 201,
	LIGHT_TAIL_RIGHT = 202,
	UNKNOWN_ENTITY = 9999
};

enum class LED_MODE
{
	LIGHT_OFF = 0,
	LIGHT_ON = 1,
	LIGHT_BLINK = 2,
	LIGHT_GRAB = 10,
	LIGHT_PUT = 11,
	LIGHT_BRAKE = 100,
	LIGHT_TURN = 101,
	LIGHT_SELECT = 102
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
