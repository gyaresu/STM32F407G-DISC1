#ifndef _MVC_MODEL_
#define _MVC_MODEL_

#include "stdint.h"
#include "arm_math.h"

#define FLAG_DIRECTION 			0x01
#define FLAG_VALUE_TYPE			0x02
#define FLAG_MACHINE_STATE	0x04

typedef enum _direction_ {FORWARD = 0x00, BACKWARD = 0x01} Direction;
typedef enum _value_type_ {DIFFERENCE = 0x00, EXACT = 0x02} ValueType;
typedef enum _machines_state_ {STOPED = 0x00, MOVED = 0x04} State;

typedef struct _car_property_ {
	
	int8_t LeftWheelVelocity;
	
	int8_t RightWheelVelocity;
	
	uint8_t flags;
	
	uint16_t allign;
	
} Car;

//This three function must be implemented with respect to using library in the project (HAL, StdPeriph etc.)
void CarCalibrate(float32_t *x, float32_t *y);
uint16_t readAnalogValueX(void);
uint16_t readAnalogValueX(void);

void CarControlInit(void);
uint16_t GetValueX_diff(int16_t *currentX);
uint16_t GetValueY_diff(int16_t *currentY);

#endif
