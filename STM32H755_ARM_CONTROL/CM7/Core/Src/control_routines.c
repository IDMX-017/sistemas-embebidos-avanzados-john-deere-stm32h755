/*
 * control_routines.c
 *
 *  Created on: Nov 21, 2024
 *      Author: Usuario
 */

#include "main.h"
#include "robot_control.h"
#include "control_routines.h"



typedef enum {
   mov_front,
   mov_back,
   mov_left,
   mov_right
} MovementType; //Usable para Fuz Control de leds

typedef struct {
  MovementType movement;
  int time;
  int angle;
  int speed;
} MovementRoutine; //Tal vez innecesario por la estructura de control.

void Servo_Init(TIM_HandleTypeDef htim13)
{
	if (HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1) != HAL_OK)
	  {
		  Error_Handler();
	  }
	__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 1500);
}

void Motor_Init(TIM_HandleTypeDef htim14)
{
	  if (HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) != HAL_OK)
	  {
	  	  Error_Handler();
	  }
	  __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 1500); //Asegurarse de usar este Timer
}

void Motor_SetSpeedV3(RobotControl *control)
{
	float32_t speed = control->v; //Obtiene la velocidad deseada en la estructura de control
	float32_t PWM_Pulse = 1500.0f + (fabsf(speed*5.0f*100.0f));
	TIM14->CCR1 = (uint32_t)saturate(PWM_Pulse, MIN_PWM, MAX_PWM);
}

void Servo_SetAngleV3(RobotControl *control)
{
	float32_t angle = control->delta; //El ángulo ya viene saturado desde robot_control.c
	TIM13->CCR1 = (uint32_t)(1500.0f + fabsf(angle*9.0f));

}

/* Sustituida por la implementacion del PurePursuit */

//void Motor_SetSpeed2(int speed, int delayTime) // Range from -100 to 100 //Tal vez innecesario el uso del tiempo para actualización constante
//{
//	const uint32_t change = 10;
//	uint32_t delay = 10;
//	uint32_t x = TIM14->CCR1;
//
//	speed *= -1;
//
//	if(speed > 100) speed = 100;
//	else if (speed < -100) speed = -100;
//
//	int newSpeed = 1500 + (speed*5);
//
//	while(abs(newSpeed - x) > 10)
//	{
//		if (newSpeed > x) x += change;
//		else if (newSpeed < x) x -= change;
//
//		TIM14->CCR1 = (uint32_t) x;
//		osDelay(delay);
//	}
//	osDelay(delayTime);
//}
/*
void Motor_Move(MovementType direction, int delayTime, int angle)
{
   switch(direction)
   {
       case mov_front:
           Motor_SetSpeed2(-100,delayTime); // Ajusta la velocidad
           break;
       case mov_back:
           Motor_SetSpeed2(100,delayTime);
           break;
       case mov_left:
           Servo_SetAngle(angle);
           Motor_SetSpeed2(100,delayTime);
           break;
       case mov_right:
           Servo_SetAngle(angle);
           Motor_SetSpeed2(100,delayTime);
           break;
       default:
    	   Motor_SetSpeed2(0,delayTime);
    	   break;
   }
}
*/

/*
void Motor_Stop(void)
{
   Motor_SetSpeed2(0,1000);
}


void ExecuteMovement(MovementRoutine *movement)
{
  switch (movement -> movement)
  {
  case mov_front:
	  Servo_SetAngle(movement->angle);
	  Motor_SetSpeed2(movement->speed, movement->time);
    break;
  case mov_back:
	  Servo_SetAngle(movement->angle);
	  Motor_SetSpeed2(movement->speed, movement->time);
	  break;
  case mov_left:
	  Servo_SetAngle(movement->angle);
	  Motor_SetSpeed2(movement->speed, movement->time);
	  break;
  case mov_right:
	  Servo_SetAngle(movement->angle);
	  Motor_SetSpeed2(movement->speed, movement->time);
    break;
  default:
    break;
  }
}
*/


//Sustituida por la implementacion del PID
/*
// Function to set the servo position (angle)
void Servo_SetAngle(int angle)		// Angle betweeen
{
	const uint32_t change = 10;
	uint32_t delay = 10;
	uint32_t x = TIM13->CCR1;


	if(angle > 60) angle = 60;
	else if (angle < -50) angle = -50;

	int newAngle = 1500 + (angle*9);

	while(abs(newAngle - x) > 10)
	{
		if (newAngle > x) x = x + change;
		else if (newAngle < x) x = x - change;

		TIM13->CCR1 = (uint32_t) x;
		osDelay(delay);
	}
	osDelay(2000);
}
*/
