#include "model.h"

Car CarModel;

uint16_t currentMiddleX, currentMiddleY;

__weak void CarCalibrate(float32_t *x, float32_t *y)
{
}

__weak uint16_t readAnalogValueX(void)
{
	return 0xFFFF;
}	

__weak uint16_t readAnalogValueY(void)
{
	return 0xFFFF;
}

uint8_t absolute(int8_t input)
{
	return (~input) + 1;
}

uint16_t GetValueX_diff(int16_t *currentX)
{

	int8_t direction = readAnalogValueX() - currentMiddleX;
	
	*currentX = direction - CarModel.LeftWheelVelocity;
	
	CarModel.LeftWheelVelocity = CarModel.LeftWheelVelocity + *currentX;
	CarModel.RightWheelVelocity = CarModel.RightWheelVelocity + *currentX;

	return ((*currentX > 2) || (*currentX < -2)) ? 0xFF00 : 0;
}

uint16_t GetValueY_diff(int16_t *currentY)
{
	static int8_t prevTurn = 0;
	
	int8_t turn = readAnalogValueY() - currentMiddleY;
	
	*currentY = turn - prevTurn;
	
	CarModel.RightWheelVelocity = CarModel.RightWheelVelocity + (turn - prevTurn);
	
	prevTurn = turn;
	
	return ((*currentY > 2) || (*currentY < -2)) ? 0xFF : 0;
}

void CarControlInit(void)
{
	float32_t x, y;
	
	CarCalibrate(&x, &y);
	
	currentMiddleX = (uint16_t )x;
	currentMiddleY = (uint16_t )y;
	
	CarModel.LeftWheelVelocity = 0;
	CarModel.RightWheelVelocity = 0;
	CarModel.flags = 0;
}
