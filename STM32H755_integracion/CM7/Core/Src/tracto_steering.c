/*
 * tracto_steering.c
 *
 *  Created on: Nov 14, 2024
 *      Author: Ivan de Mexico
 */
#include "main.h"

void TurningSetAngle(float angle) {
    // Ajustar el ángulo al rango permitido
    if (angle < -90.0f) angle = -90.0f;
    if (angle > 90.0f)  angle = 90.0f;

    int pulse;
    if (angle <= 0.0f) {
        // Mapear ángulos de -90 a 0 grados
        pulse = (int)((80.0f / 90.0f) * (angle + 90.0f) + 130.0f);
    } else {
        // Mapear ángulos de 0 a 90 grados
        pulse = (int)((50.0f / 90.0f) * angle + 210.0f);
    }
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, pulse);
}

void SetMotorSpeed(float speed)
{
	//Limitar el valor de speed
	if (speed < 0.0f) speed = 0.0f;
	if (speed > 1.0f) speed = 1.0f;

    // Ajusta el valor de CCR1 en función de la velocidad deseada
    TIM14->CCR1 = (uint32_t)((63999 * 0.05f) + (63999 * 0.05f * speed));
}

uint32_t ProportionalStateMachine (uint32_t x_target, uint32_t x_current)
{
	typedef enum {IDLE, THROTTLE,TOP_SPEED, BRAKING} Status;

	static Status TractorStatus;

	switch (TractorStatus)
	{
	case IDLE:
		HAL_GPIO_Toggle(LD1_GPIO_Port, LD1_Pin);
		TractorStatus = ((x_target - x_current) != 0) ? THROTTLE : IDLE;
		return 0;

	case THROTTLE:
		if (x_current >= x_target / 3)
		{
		   TractorStatus = TOP_SPEED;
		}
		else
		{
			return (((x_target - x_current) * 100) / x_target/3 > 100) ? 100 : ((x_target - x_current) * 100) / x_target/3;
		}

	case TOP_SPEED:
		if (x_current >= x_target/2)
		{
			TractorStatus = BRAKING;
		}
		return 100;

	case BRAKING:
		if (x_current >= x_target) {
			TractorStatus = IDLE;
		}
		return ((x_target - x_current) * 100) / x_target;


	default:
		TractorStatus = IDLE;
	}

}

