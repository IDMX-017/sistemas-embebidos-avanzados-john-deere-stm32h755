/*
 * pure_pursuit.c
 *
 *  Created on: Nov 18, 2024
 *      Author: Ivan de Mexico
 */

#include <stdio.h>
#include <math.h>
#include "robot_control.h"

arm_pid_instance_f32 pid_delta;
static Status Robot_status = IDLE;

void robot_init(RobotState *state)
{
    state->x = 0.0f;
    state->y = 0.0f;
    state->theta = 0.0f;
    state->v = 0.0f;
    state->delta = 0.0f;
}

/* Function for Value Saturation */
float32_t saturate(float32_t value, float32_t min, float32_t max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

float32_t normalize_angle(float32_t angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

void PID_Init(arm_pid_instance_f32 *pid, float32_t Kp, float32_t Ki, float32_t Kd)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	arm_pid_init_f32(pid, 1);
}

float32_t PID_Angle(arm_pid_instance_f32 *pid_delta, float32_t delta_desired, float32_t delta_actual)
{
    //float32_t error = normalize_angle(delta_desired - delta_actual);
	return arm_pid_f32(pid_delta, normalize_angle(delta_desired - delta_actual));
}

float32_t ANGLE_OBJECTIVE(RobotState *state, float32_t x_desired, float32_t y_desired)
{
	float32_t angle_radian = (float32_t)(atan2f(y_desired - state->y, x_desired - state->x));
	if (angle_radian > PI) angle_radian -= 2.0f * PI;
	    if (angle_radian < -PI) angle_radian += 2.0f * PI;
	//float32_t angle_objective = (180.0f/PI)*angle_radian;
	return angle_radian;
}

float32_t calc_vector_length(float32_t x1, float32_t y1, float32_t x2, float32_t y2)
{
	float32_t result;
	float32_t dx = x2 - x1;
	float32_t dy = y2 - y1;
	arm_sqrt_f32(dx * dx + dy * dy, &result);
	return result;
}

void PURE_PURSUIT(RobotState *state, float32_t *desired_delta, float32_t *desired_v, Path *path)
{
	uint32_t target_index = path->current_index;
	float32_t dx, dy, distance;

	float32_t waypoint_tolerance = 0.01f;

	while (target_index < path->length) {
		// Obtener Waypoint Actual
		Waypoint current_waypoint = path->waypoints[target_index];

		// Calcular Distancia al Waypoint actual
		dx = current_waypoint.x - state->x;
		dy = current_waypoint.y - state->y;
		arm_sqrt_f32(dx * dx + dy * dy, &distance);

		if (distance < waypoint_tolerance) {
			/* Avanzar al siguiente waypoint */
			path->waypoints[target_index].flag = 1;
			target_index++;
			path->current_index = target_index;
		} else {
			/* Si el waypoint no se ha alcanzado, salir del bucle */
			break;
		}
	}

	// Verificar si se ha recorrido toda la trayectoria
    if (target_index >= path->length) {
        *desired_v = 0.0f;
        *desired_delta = 0.0f;
        Robot_status = IDLE;
        return;
    }

    // Si no se ha completado la trayectoria continua
    Waypoint current_waypoint = path->waypoints[target_index];
    Waypoint next_waypoint = (target_index + 1 < path->length) ? path->waypoints[target_index + 1] : current_waypoint;

    float32_t distance_to_next_waypoint = calc_vector_length(current_waypoint.x, current_waypoint.y, next_waypoint.x, next_waypoint.y);

    float32_t target_x, target_y;
	if (distance_to_next_waypoint < LD)
	{
		// Si LD es mayor que la distancia al siguiente waypoint, se usa directamente el siguiente waypoint
		target_x = next_waypoint.x;
		target_y = next_waypoint.y;
	} else {
		// Interpolar dinámicamente para calcular el punto de mira
		float32_t t = LD / distance_to_next_waypoint;
		target_x = (1 - t) * current_waypoint.x + t * next_waypoint.x;
		target_y = (1 - t) * current_waypoint.y + t * next_waypoint.y;
	}

	/*
    float32_t vector_magnitude = distance;
        if (vector_magnitude < 1e-6f) vector_magnitude = 1e-6f;

	float32_t unit_dx = dx / vector_magnitude;
	float32_t unit_dy = dy / vector_magnitude;
	*/
	/* CALCULO DE LEAD POINT */
	/*
	target_x = state->x + unit_dx * LD;
	target_y = state->y + unit_dy * LD;
	*/

	/* Calcular el vector hacia el siguiente waypoint */
    dx = target_x - state->x;
    dy = target_y - state->y;
    float32_t angle_to_target = (float32_t)atan2f(dy, dx);

    float32_t alpha = normalize_angle(angle_to_target - state->theta);

	float32_t gamma = (2.0f*arm_sin_f32(alpha))/LD;

	*desired_delta = (float32_t)atan(L*gamma);
    *desired_delta = saturate(*desired_delta, -DELTA_MAX, DELTA_MAX);

    float32_t v_base = 0.5f;  // Velocidad base
    float32_t k = 1.0f;       // Constante de ajuste
    *desired_v = v_base / (1.0f + k * (float32_t)fabsf(gamma));

    if (target_index == path->length - 1) {
	float32_t distance_to_goal = calc_vector_length(
			state->x, state->y, current_waypoint.x, current_waypoint.y
			);
		*desired_v *= (distance_to_goal / LD);  // Reducir progresivamente la velocidad
	}

	/* Saturar la Velocidad Deseada */
	*desired_v = saturate(*desired_v, V_MIN, V_MAX);
}

void catmull_rom_spline(Waypoint p0, Waypoint p1, Waypoint p2, Waypoint p3, float32_t t, Waypoint *result) {
    float32_t t2 = t * t;
    float32_t t3 = t2 * t;

    result->x = 0.5f * ((2.0f * p1.x) +
                        (-p0.x + p2.x) * t +
                        (2.0f * p0.x - 5.0f * p1.x + 4.0f * p2.x - p3.x) * t2 +
                        (-p0.x + 3.0f * p1.x - 3.0f * p2.x + p3.x) * t3);

    result->y = 0.5f * ((2.0f * p1.y) +
                        (-p0.y + p2.y) * t +
                        (2.0f * p0.y - 5.0f * p1.y + 4.0f * p2.y - p3.y) * t2 +
                        (-p0.y + 3.0f * p1.y - 3.0f * p2.y + p3.y) * t3);
}

void PURE_PURSUIT_Spline(RobotState *state, RobotState *state_prev, float32_t *desired_delta, float32_t *desired_v, Path *path, float32_t *t, float32_t *segment_length)
{
	uint32_t target_index = path->current_index;
	float32_t dx, dy, distance;


	float32_t waypoint_tolerance = 0.01f;

	/* Calcular la distancia al waypoint actual */
	dx = path->waypoints[target_index].x - state->x;
	dy = path->waypoints[target_index].y - state->y;
	arm_sqrt_f32(dx * dx + dy * dy, &distance);

	/* Verificar si se ha alcanzado el waypoint actual */
	if (distance < waypoint_tolerance)
	{
		/* Avanzar al siguiente waypoint */
		if (target_index < path->length - 1) {
			path->current_index++;
			*t = 0.0f; // Reiniciar t para el nuevo segmento

			/* Actualizar la longitud del segmento */
			Waypoint p1 = path->waypoints[path->current_index];
			Waypoint p2 = path->waypoints[path->current_index + 1];

			dx = p2.x - p1.x;
			dy = p2.y - p1.y;
			arm_sqrt_f32(dx * dx + dy * dy, segment_length);
		} else {
		/* Último waypoint alcanzado */
		*desired_v = 0.0f;
		*desired_delta = 0.0f;
		Robot_status = IDLE;
		return;
		}
	}

    dx = state->x - state_prev->x;
    dy = state->y - state_prev->y;
    float32_t distance_traveled;
    arm_sqrt_f32(dx * dx + dy * dy, &distance_traveled);

    /* Evitar división por cero */
    if (*segment_length < 1e-6f) {
        *segment_length = 1e-6f;
    }

    /* Calcular t_increment */
    float32_t t_increment = distance_traveled / (*segment_length);

    /* Actualizar t */
    *t += t_increment;
    if (*t > 1.0f) *t = 1.0f; // Limitar t a 1.0

    /* Obtener los puntos de control */
    Waypoint p0 = (target_index == 0) ? path->waypoints[target_index] : path->waypoints[target_index - 1];
    Waypoint p1 = path->waypoints[target_index];
    Waypoint p2 = path->waypoints[target_index + 1];
    Waypoint p3 = (target_index + 2 < path->length) ? path->waypoints[target_index + 2] : path->waypoints[target_index + 1];

    Waypoint target_point;
    catmull_rom_spline(p0, p1, p2, p3, *t, &target_point);
	*t += t_increment;

    /* Calcular el ángulo hacia el punto objetivo */
    dx = target_point.x - state->x;
    dy = target_point.y - state->y;
    float32_t angle_to_target = (float32_t)atan2f(dy, dx);

    /* Calcular el ángulo de dirección deseado */
    float32_t alpha = normalize_angle(angle_to_target - state->theta);

    /* Calcular la curvatura */
    float32_t gamma = (2.0f * arm_sin_f32(alpha)) / LD;

    /* Calcular desired_delta */
    *desired_delta = (float32_t)atanf(L * gamma);
    *desired_delta = saturate(*desired_delta, -DELTA_MAX, DELTA_MAX);

    float32_t v_base = 0.5f;  // Velocidad base
    float32_t k = 1.0f;       // Constante de ajuste
    *desired_v = saturate(v_base / (1.0f + k * (float32_t)fabsf(gamma)), 0.0f, V_MAX);
}









