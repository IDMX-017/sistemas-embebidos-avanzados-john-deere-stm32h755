/*
 * control_routines.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Usuario
 */

#ifndef INC_CONTROL_ROUTINES_H_
#define INC_CONTROL_ROUTINES_H_

//Macros utiles
#define MAX_PWM 2000.0f
#define MIN_PWM 1000.0f

void Servo_Init(TIM_HandleTypeDef htim13);
void Motor_Init(TIM_HandleTypeDef htim14);
void Motor_SetSpeedV3(RobotControl *control);
void Servo_SerAngleV3(RobotControl *control);

#endif /* INC_CONTROL_ROUTINES_H_ */
